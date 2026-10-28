# QAM Communication Project – ESP32-S3 EduBoard2

## Project Overview

This project is part of the **Data Transmission / Modulation / Demodulation** module at Juventus Technikerschule HF.

The goal is to develop a communication system between two independent embedded systems. Digital data should be transmitted over a communication channel using **Quadrature Amplitude Modulation (QAM)**.

The main requirements are:

- Implement at least **QAM-4**
- Perform modulation and demodulation mainly in software
- Transfer digital data between two independent systems
- Use packet-based communication
- Include basic error detection
- Analyse symbol rate, bit rate and required channel bandwidth
- Compare theoretical calculations with the practical implementation
- Higher-order QAM, such as QAM-16, should also be considered conceptually

The target platform is the **EduBoard2 with an ESP32-S3**.

---

## Current Development Approach

Before implementing the complete QAM signal path, the communication protocol and packet handling were tested independently.

For this first development step, **I2C was used as a temporary transport layer** between two EduBoard2 systems.

This makes it possible to verify:

- Sensor data acquisition
- Packet generation
- Gray-code conversion
- Data transmission
- Packet reception
- Gray-code decoding
- Checksum verification
- Reconstruction of the original sensor value

without having to debug the QAM modulation and demodulation at the same time.

The I2C connection is therefore only used as an intermediate development and test step.

---

## First Test Setup

The sender reads the temperature from the onboard **TMP112 temperature sensor**.

The measured value is converted into a 12-bit value and packed into the following structure:

```text
START | DATA | CHECK
12 Bit  12 Bit  12 Bit
```

The three fields are converted into Gray code before transmission.

### Sender

```text
TMP112
   ↓
12-bit temperature value
   ↓
START / DATA / CHECK
   ↓
Gray-code encoding
   ↓
I2C Master
```

### Receiver

```text
I2C Slave
   ↓
START / DATA / CHECK
   ↓
Gray-code decoding
   ↓
Start sequence validation
   ↓
Checksum validation
   ↓
Temperature reconstruction
   ↓
Display / Serial output
```

The complete packet is transferred as 6 bytes:

```text
START_H
START_L
DATA_H
DATA_L
CHECK_H
CHECK_L
```

---

## I2C Test Result

The first communication test using I2C was **successful**.

The receiver is able to:

- Receive the complete packet
- Separate the three 12-bit fields
- Decode the Gray-coded values
- Detect the defined start sequence
- Verify the checksum
- Reconstruct the original temperature value
- Display the received temperature
- Count valid, invalid and dropped packets

This confirms that the packet format and the basic software data path are working correctly.

The current data path is:

```text
TMP112
   ↓
Packet generation
   ↓
Gray-code encoding
   ↓
I2C
   ↓
Gray-code decoding
   ↓
Packet validation
   ↓
Temperature
```

---

## Next Step: QAM-4

The next development step is to replace the temporary I2C transport with the actual QAM signal path.

The planned transmitter path is:

```text
TMP112
   ↓
Packet generation
   ↓
Bit stream
   ↓
QAM-4 mapping
   ↓
QAM signal generation
   ↓
DAC
   ↓
Analog transmission channel
```

The planned receiver path is:

```text
Analog transmission channel
   ↓
ADC
   ↓
QAM-4 demodulation
   ↓
I/Q evaluation
   ↓
Symbol-to-bit conversion
   ↓
Packet decoding
   ↓
Temperature
```

For the first QAM test, the DAC output and ADC input will be connected directly by wire.

Once the wired QAM transmission is working reliably, the physical transmission channel can be extended to a wireless solution.

---

## QAM-4 Principle

QAM-4 transfers **2 bits per symbol**.

Example mapping:

```text
00 -> I = -1, Q = -1
01 -> I = -1, Q = +1
10 -> I = +1, Q = -1
11 -> I = +1, Q = +1
```

The transmitted signal is generated from the in-phase and quadrature components:

```text
s(t) = I · cos(ωt) + Q · sin(ωt)
```

On the receiver side, the sampled ADC signal is multiplied with sine and cosine reference signals to reconstruct the I and Q components.

These values are then used to determine the transmitted QAM symbol and recover the original bits.

---

## Development Milestones

### Milestone 1 – Basic I2C Receiver

- Configure ESP32-S3 as I2C slave
- Receive 6-byte packets
- Split packet into START, DATA and CHECK fields
- Output received values over the serial console

### Milestone 2 – Packet Decoding and Validation

- Decode Gray code
- Validate start sequence
- Validate checksum
- Reconstruct the 12-bit temperature value
- Add basic error counters

### Milestone 3 – Receiver Display and Status Handling

- Display received data on the EduBoard2 LCD
- Show binary and hexadecimal packet values
- Display decoded temperature
- Detect missing valid packets
- Add packet error and drop statistics

---

## Planned Final Signal Path

```text
                     TRANSMITTER

TMP112
  ↓
Packet Encoder
  ↓
QAM-4 Modulator
  ↓
DAC
  ↓
Transmission Channel
  ↓
ADC
  ↓
QAM-4 Demodulator
  ↓
Packet Decoder
  ↓
Temperature

                      RECEIVER
```

The I2C implementation serves as a validated baseline for the packet layer. From this point onward, the main focus of the project is the implementation and testing of the actual **QAM-4 modulation and demodulation**.

### Milestone 4 – Basic ADC/QAM Demodulation
The I2C receiver was replaced by a continuous ADC input.

The first QAM demodulation implementation contains:
- Continuous ADC sampling
- DMA based sample acquisition
- DC offset estimation and removal
- Simple carrier detection
- Sample buffering
- Basic I/Q correlation using sine and cosine reference tables
- QAM-4 symbol decisions
- Reconstruction of the 36-bit packet
- Existing Gray-code decoding and packet validation
- Existing temperature display and error counters
At this stage the receiver still assumes that the captured sample block is approximately aligned with the transmitted QAM symbols.

No automatic phase or symbol timing correction is implemented yet.
