
# ESP32FMRadio: FM Transmission Using APLL andI2S MCLK on ESP32

**ESP32FMRadio** is a project that turns an ESP32 microcontroller into a mini FM radio transmitter by
exploiting the ESP32’s Audio PLL (APLL) and I2S peripheral’s master clock (MCLK) output. Instead of
using PWM or DAC methods, this approach directly generates an RF carrier (in the FM band, e.g.
~100 MHz) on a GPIO pin using the ESP32’s high-frequency clock capabilities. The carrier is then
**frequency modulated (FM)** with an audio signal (in this case, an 8 kHz sampled audio of the famous
Rick Astley song) to produce a broadcastable FM radio signal. The result is a simple but functional FM
transmitter, achieved purely with the ESP32’s internal hardware (no external RF transmitter chip). This
write-up explains in detail how the APLL and I2S MCLK are configured and utilized to achieve FM
transmission.

## The ESP32’s Audio PLL (APLL) and Frequency Generation
The **Audio PLL (APLL)** in the ESP32 is a dedicated phase-locked loop designed to generate precise high-
frequency clocks for audio applications (like I2S or DAC sampling). Unlike the main CPU clock PLLs, the
APLL can be tuned to arbitrary frequencies in a certain range, making it ideal for generating a custom
frequency for our FM carrier.

- **APLL Output Formula:** The frequency output by the APLL is determined by a formula involving the 40 MHz crystal oscillator (XTAL) and programmable dividers. In essence, the APLL allows setting an integer and fractional divider (often represented by ```sdm2``` , ```sdm1``` , ```sdm0``` for the fractional synthesizer and an ```o_div``` output divider). The Technical Reference Manual indicates the formula (for ESP32 rev.0 chips) as:
![equation](https://latex.codecogs.com/svg.image?$$f_{\text{out}}=f_{\text{xtal}}\times\frac{4&plus;\text{sdm2}&plus;\dfrac{\text{sdm1}}{2^{8}}&plus;\dfrac{\text{sdm0}}{2^{16}}}{2\times\bigl(o_{\text{div}}&plus;2\bigr)}$$)

This means the base multiplier is ```4 + sdm_fractional``` divided by ```2*(o_div+2)```. For
example, with a 40 MHz crystal, choosing appropriate values (sdm2, sdm1, sdm0, o_div) can yield a
desired output. The APLL has certain locking constraints: the internal VCO (numerator part,
```f × xtal (4 + sdm2 + ...)```) must lie between 350 MHz and 500 MHz to lock properly. If it's outside this
range, the PLL won’t stabilize (too low or too high).

- **Generating a 100 MHz Carrier:** In our case, we target an FM carrier around 100 MHz (within the commercial FM band). Using the formula, one simple valid configuration is to set ```o_div = 0``` (minimal division) and adjust ```sdm``` such that the numerator becomes 400 MHz (which divided by 4 gives 100 MHz). For instance, setting ```sdm2 = 6``` , ```sdm1 = 0``` , ```sdm0 = 0``` yields: ```fout = 40 MHz × (4 + 6 + 0 + 0)/(2 × (0 + 2)) = 40 × 10/4 = 100 MHz``` via the ```fm_calc_apll()``` function to handle different target frequencies and ensure the lock conditions are met (finding the smallest o_div that keeps the VCO ≥350 MHz, etc.).

- **Frequency Resolution:** The APLL offers fine frequency control via the 16-bit fractional part (sdm0) of the synthesizer. The smallest step (1 LSB of sdm0) corresponds to a few tens or hundreds of Hz change in output frequency, depending on o_div. In our 100 MHz example with o_div=0, one LSB of the 16-bit fraction changes the output by roughly ~152 Hz (since 1 step out of 2^16 of 400 MHz VCO is ~6.1 kHz at VCO, divided by 4 gives ~1.5 kHz). Actually, the code calculates this precisely: ```sb_hz = XTAL / (2*(o_div+2)*65536)```, and for o_div=0 that is ```40e6/(4 ∗ 65536)```. This fine control is what allows us to modulate the frequency smoothly with audio.

## Using I2S MCLK to Output the RF Carrier
The ESP32’s **I2S (Inter-IC Sound)** peripheral is typically used for digital audio data, but it also provides a **Master Clock (MCLK)** output intended to drive external DAC/ADC chips. We can repurpose this MCLK output to drive our antenna (GPIO pin) with the high-frequency carrier from the APLL.

- **Configuring I2S for APLL MCLK:** In the code, the I2S peripheral is initialized in TX master mode, but we don’t actually send any meaningful audio data out – we only care about the clock. The important config is setting ```.use_apll = true``` and specifying ```.fixed_mclk = FM_CARRIER_HZ``` (which is 100,000,000 Hz in our case). According to Espressif’s documentation, when ```use_apll``` is true and ```fixed_mclk``` is set, the I2S driver will configure the APLL to produce that exact MCLK frequency. In other words, we request a 100 MHz clock and the ESP32’s I2S/APLL system takes care of tuning the PLL to achieve it (under the hood, it’s doing essentially the same calculation described in the APLL section).

- **Routing MCLK to a GPIO:** By default, the I2S MCLK might not be output to a pin until we route it. The code uses ```PIN_FUNC_SELECT``` and some register settings to map the I2S0 MCLK signal to a physical GPIO (GPIO0 in this project). Specifically, it selects the function “CLK_OUT1” on GPIO0 and configures the ```CLK_OUT1``` source to be I2S0 MCLK (by writing 0 to the CLK_OUT1 field of the PIN_CTRL register). After this, we set GPIO0 as an output. The result is that **GPIO0 now continuously outputs a square wave at 100 MHz** (the I2S MCLK). This is our unmodulated carrier. A short piece of wire on GPIO0 can act as a very low-power antenna to broadcast the signal to a short distance.

## Frequency Modulation via APLL Adjustments
With a 100 MHz carrier being output, the next step is to **modulate** it with audio. Frequency Modulation
means we vary the carrier frequency slightly up and down in proportion to the audio waveform.
Achieving this on the ESP32 is done by dynamically tweaking the APLL frequency on the fly:

- **Calculating Deviation Steps:** The code defines a maximum frequency deviation ```MAX_DEV_HZ``` (in our case 62,000 Hz, approximately ±62 kHz). In commercial FM broadcast, ±75 kHz is the standard max deviation for full modulation, so 62 kHz is a bit lower (possibly chosen for safety or hardware limitation reasons). Using the earlier computed ```lsb_hz``` (Hz per PLL LSB step), the code computes ```dev_lsb = round(MAX_DEV_HZ / lsb_hz)``` This tells how many fractional steps correspond to the desired frequency swing. For example, if one LSB ≈152 Hz, to get ±62,000 Hz you need roughly ±407 LSB steps (62000/152 ≈ 408). The code indeed computes and stores this as ```c.dev_lsb```.

- **Centering the Frequency:** The carrier’s base frequency is set by ```base_sdm0``` (the 16-bit base fractional value for the PLL to produce exactly 100 MHz). To allow symmetrical modulation up and down, the code ensures that ```base_sdm0``` is not too close to 0 or 65535. If the base value is less than ```dev_lsb```, it adds an offset (essentially shifting the center frequency slightly) so that we can go ±```dev_lsb``` without hitting the 0 or max boundary of the 16-bit range. This is important for frequencies at the low end of the band (e.g., 87 MHz might need such an offset because the natural base_sdm0 could be small). After this, the APLL is initialized with these base coefficients.

- **High-Speed Timer Interrupt:** To actually perform modulation, an **ESP32 high-resolution timer** is used. The code sets up a periodic timer ( esp_timer ) to trigger an interrupt/callback at 8000 Hz – this is exactly the sample rate of our audio. In the timer callback (marked IRAM_ATTR for speed), the next audio sample is read from the array. This audio is an 8-bit unsigned PCM (values 0–255) of the song. The code subtracts 128 to convert it to a signed value (-128 to +127). This yields the audio amplitude at that sample, where 0 corresponds to no frequency deviation (carrier at base frequency) and ±127 corresponds to maximum negative or positive deviation.

- **Applying the Frequency Change:** The sample (now in range -128 to +127) is scaled by ```dev_lsb```. Essentially: ```delta = (audio_sample * dev_lsb) >> 7```. The ```>>7``` is dividing by 128, which aligns with the max amplitude 127 ~ ```dev_lsb```. In effect, when audio_sample is 127 (max), ```delta``` ≈ + ```dev_lsb```; when -128, delta ≈ - ```dev_lsb```. This ```delta``` is the amount by which we will shift the PLL’s fractional part from the base. The code then calls ```fm_set_deviation(delta)``` which does: ```sdm0 = base_sdm0 + delta```, clamps it between 0 and 65535, and then uses ```clk_ll_apll_set_config(...)``` to update the APLL registers with the new sdm0 (keeping sdm1, sdm2, o_div constant). This directly **nudges the APLL frequency** up or down in proportion to the audio signal at that instant. The PLL reacts almost instantly to the changes (within microseconds), thus modulating the output frequency in realtime.

This mechanism is effectively performing FM modulation: the instantaneous frequency of the 100 MHz
output is being shifted slightly higher or lower depending on the input waveform amplitude at each
moment. A nearby FM radio receiver tuned to ~100 MHz will interpret these frequency changes as the
original audio.

# Audio Source and Preparation
The project uses a short audio clip (Rick Astley’s "Never Gonna Give You Up") to demo the transmission.
Key details about the audio handling:

- **Embedded Audio Data:** Instead of streaming from SD card or external source, the audio is compiled into the firmware. A WAV file (8-bit mono, 8000 Hz) was converted into a C header array ( rickroll.h ). The tool xxd was used to dump the raw PCM bytes of the WAV into a C array format. This makes the audio readily available in memory. At runtime, no file I/O is needed; we just read from the array. The array values range 0–255 (8-bit unsigned samples).

- **Sample Rate and Timer Sync:** The WAV is 8000 samples per second, so the timer is configured to 125 μs period (which is 1/8000 s). This ensures we "play" the samples at the correct speed. If this rate didn’t match, the audio would be distorted (e.g., wrong pitch). The ESP32’s esp_timer is accurate for this purpose. By keeping the modulation update in lockstep with the original sample rate, we preserve the audio timing. Essentially, the ESP32 is acting like it’s **generating an FM radio signal in real-time from PCM data**, similar to how an FM transmitter would take an audio input and modulate a carrier.

# Output and Antenna Considerations
Once the system is running, GPIO0 is effectively transmitting a radio signal. Here are some important
points about the output and its limitations:

- **Signal Characteristics:** The output is a digital square wave at ~100 MHz that is frequency- modulated. A square wave has many harmonics (odd harmonics at 3x, 5x frequencies etc.), which means the ESP32 is actually emitting not just at 100 MHz but also some energy at 300 MHz, 500 MHz, etc. Ideally, one would use an output filter or at least a small tuned antenna to focus on the fundamental frequency. In practice, a short wire will radiate the strongest at frequencies where its length is a significant fraction of the wavelength. For ~100 MHz, a quarter- wave is about 75 cm. Using a short wire (a few centimeters) will severely limit range (which is good for avoiding interference, but also means you need to be very close to the radio to receive it).

- **Range and Power:** The ESP32’s GPIO is not designed as an RF power amplifier. The transmitted power is very low (likely in the microwatt range), and the range might be only a few meters at best with a short wire. This is fine for demonstration and keeps within what is generally permissible as unintentional radiator emissions. It’s effectively a **very low-power micro FM transmitter**. One should still be cautious and use it only in a controlled environment, as broadcasting on FM frequencies without a license can be illegal if power or range is significant. In our case, the range is minimal.

- **Hardware Requirements:** This project specifically requires the original ESP32 chip (ESP32 D0WD, e.g., found in WROOM/WROVER modules) because it has the APLL feature. The newer variants like ESP32-S2, S3, C3 do **not** have the APLL hardware, so they cannot generate an arbitrary high-frequency MCLK like this. The code even explicitly errors out at compile time if not on an ESP32 original, noting that those series don’t possess APLL. Therefore, one must use an ESP32 (original series) for this to work. Also, using GPIO0 as the output pin is convenient on dev boards, but remember GPIO0 is a strapping pin (it selects boot mode at reset). This means you should avoid forcing GPIO0 low at reset, or the ESP32 might not boot. In our use, once running, GPIO0 is set as an output driving the RF signal.

- **Stability:** The APLL provides a fairly stable frequency with low jitter (it’s meant for audio clocks). By using it, we get a cleaner tone than, say, trying to bit-bang or use a generic PWM. According to some experiments, the APLL-based MCLK has low jitter and can produce a clean carrier . This ensures our FM signal has decent sound quality (as good as 8 kHz mono allows) without excessive noise. The stability was also aided by using the high-res timer for updates. If the timing or PLL update was inconsistent, it would introduce noise or distortion in the received audio.