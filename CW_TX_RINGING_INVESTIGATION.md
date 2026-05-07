# CW TX "Ringing" Investigation

**Status**: Under observation — not reproducible on all units. Possibly unit-specific or settings-dependent.  
**Reported by**: Beta tester (not reproducible on primary dev unit)

---

## Symptom

On-air CW sounds "melodic" or has an audible ringing quality rather than a clean carrier. The signal has what sounds like additional FM deviation or audio modulation on the carrier — "very melodic instead of a solid carrier."

## Suspected Contributing Factors

Three code-level issues were identified in the BK4829 TX path that could cause or amplify this symptom. None have been confirmed as the root cause on any specific unit.

---

### Issue 1 — MIC_ADC active during first CW element (BK4829 only)

`RADIO_SetTxParameters()` calls `BK4819_PrepareTransmit()` (bk4829.c), which calls `BK4819_ExitTxMute()` then `BK4819_TxOn_Beep()`. The `TxOn_Beep()` implementation in bk4829.c has no CW-specific path and writes `REG_30 = 0xC1FE`, which includes `ENABLE_MIC_ADC` (bit 2). The mic ADC remains live while the PA ramps up for the first element of each TX session (~4ms window).

The BK4819 (older chip, UV-K5) `BK4819_TxOn_Beep()` has an explicit CW branch that clears `ENABLE_MIC_ADC`:
```c
// bk4819.c CW path (for reference):
BK4819_WriteRegister(BK4819_REG_30,
    (0xC1FE | BK4819_REG_30_ENABLE_AF_DAC | BK4819_REG_30_ENABLE_RX_DSP)
    & ~BK4819_REG_30_ENABLE_MIC_ADC);
```
The bk4829.c version omits this and takes no CW-specific action.

**Effect**: Ambient audio or room noise could be transmitted as FM during the first-element ramp-up on affected units.

**Relevant code**: `App/driver/bk4829.c` — `BK4819_TxOn_Beep()`

---

### Issue 2 — AF = BASEBAND2 active when PA turns on (every element after the first)

`RADIO_CW_Suspend()` sets `BK4819_SetAF(BK4819_AF_BASEBAND2)` when dropping the carrier (for smooth RX re-enable). On the next key-down, `RADIO_CW_BeginResume()` currently operates in this order:

1. `BK4819_SetupPowerAmplifier(TXP, freq)` — **carrier goes live**
2. `BK4819_SetScrambleFrequencyControlWord(...)` — tone freq set
3. `BK4819_EnableTXLink()` — REG_30 set (AF_DAC=1, DISC_MODE=1)
4. `BK4819_SetAF(BK4819_AF_ALAM)` — AF mode corrected (too late)

Between steps 1 and 4, `REG_47` still routes `BASEBAND2` (SSB demodulator output) to the AF_DAC while the TX chain is live. On the BK4829, any coupling from the demodulator path into TX_DSP during this window could modulate the carrier. The window is on the order of a few SPI write cycles (~tens of microseconds), but may be enough on sensitive units.

**Effect**: Brief audio artifact at the leading edge of every element after the first.

**Relevant code**: `App/radio.c` — `RADIO_CW_BeginResume()`

**Proposed reorder (not yet applied)**:
```c
void RADIO_CW_BeginResume(void)
{
    gCW_State = CW_TRANSMITTING;
    // Configure routing BEFORE carrier comes on
    BK4819_SetAF(BK4819_AF_ALAM);
    BK4819_EnableTXLink();
    BK4819_SetScrambleFrequencyControlWord(gEeprom.CW_TONE_FREQUENCY * 10);
    // Now bring carrier up
    BK4819_SetupPowerAmplifier(gCurrentVfo->TXP_CalculatedSetting, gCurrentVfo->pTX->Frequency);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
}
```

---

### Issue 3 — FM discriminator (DISC_MODE) active during CW TX

`BK4819_EnableTXLink()` includes `BK4819_REG_30_ENABLE_DISC_MODE`. On the BK4829, if PA output couples back into the IF/RX chain (board-level shielding or layout variation between units), the discriminator demodulates any VCO pulling at key transitions and this could feed back into TX_DSP if the chip's internal routing allows it.

This is the most speculative of the three — the BK4819 CW path also had DISC_MODE on, and no ringing was reported on those units. May be a BK4829-specific internal routing difference, or could be exacerbated by PCB layout on certain production batches.

**Relevant code**: `App/radio.c` — `RADIO_CW_BeginResume()` → `BK4819_EnableTXLink()`

**Exploratory fix (not yet applied)**: Replace `BK4819_EnableTXLink()` in CW TX with an equivalent that omits `ENABLE_DISC_MODE`.

---

### Note on scale_freq multiplier

`bk4829.c` uses `scale_freq` multiplier `1353245` vs bk4819.c's `1032444` (~31% difference). This means `BK4819_SetScrambleFrequencyControlWord(CW_TONE_FREQUENCY * 10)` writes a different REG_71 value on BK4829 than on BK4819 for the same input frequency. The sidetone heard locally will be at the correct pitch (the BK4829 constant is tuned for its crystal), so this is not itself a bug — but it confirms the two chips are not pin-compatible in their frequency scaling.

---

## Next Steps (deferred)

- Confirm whether the issue is reproducible with factory-reset EEPROM on the affected unit (rules out bad settings)
- If confirmed reproducible after reset: apply Fix 2 (reorder in `RADIO_CW_BeginResume`) as the lowest-risk change and test
- If still present: apply Fix 1 (CW-specific `TxOn_Beep` path in bk4829.c)
- If still present: try Fix 3 (omit DISC_MODE in CW TX link)
