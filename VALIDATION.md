# Validation

- Python AST syntax checks passed for the new and original host scripts.
- Frame parser checks passed for 12 and 16 rows, 1/7/8192-byte chunks, partial markers, false markers, and payload bytes containing AA 55.
- Hardware upload, live USB streaming, analog measurements and force calibration have not been tested: no user hardware is connected to this environment.
- PlatformIO builds passed for nano12_old and nano16_old using atmelavr 5.3.0 and framework-arduino-avr 5.4.0.
- nano12_old: 2102 bytes flash, 186 bytes RAM. nano16_old: 2094 bytes flash, 186 bytes RAM.
- New-bootloader environments are supplied but were not separately built; hardware bootloader detection/upload was not performed.
