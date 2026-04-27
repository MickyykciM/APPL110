
#include "./I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

    #ifdef I2CDEV_IMPLEMENTATION_WARNINGS
        #if ARDUINO < 100
            #warning Using outdated Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.0.1+ with I2Cdev Fastwire implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Repeated starts conditions
            #warning - Timeout detection (some Wire requests block forever)
        #elif ARDUINO == 100
            #warning Using outdated Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.0.1+ with I2Cdev Fastwire implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Repeated starts conditions
            #warning - Timeout detection (some Wire requests block forever)
        #elif ARDUINO > 100
            #warning Using current Arduino IDE with Wire library is functionally limiting.
            #warning Arduino IDE v1.0.1+ with I2CDEV_BUILTIN_FASTWIRE implementation is recommended.
            #warning This I2Cdev implementation does not support:
            #warning - Timeout detection (some Wire requests block forever)
        #endif
    #endif

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE

    #ifdef I2CDEV_IMPLEMENTATION_WARNINGS
        #warning Using I2CDEV_BUILTIN_NBWIRE implementation may adversely affect interrupt detection.
        #warning This I2Cdev implementation does not support:
        #warning - Repeated starts conditions
    #endif

    TwoWire Wire;

#endif

I2Cdev::I2Cdev() {
}

int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {

    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {

    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(devAddr, regAddr, 1, data, timeout);
}

int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();

    #if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE)

        #if (ARDUINO < 100)

            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.send(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

                for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = Wire.receive();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }

                Wire.endTransmission();
            }
        #elif (ARDUINO == 100)

            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

                for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = Wire.read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }

                Wire.endTransmission();
            }
        #elif (ARDUINO > 100)

            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

                for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
                    data[count] = Wire.read();
                    #ifdef I2CDEV_SERIAL_DEBUG
                        Serial.print(data[count], HEX);
                        if (count + 1 < length) Serial.print(" ");
                    #endif
                }
            }
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, data, length);
        if (status == 0) {
            count = length;
        } else {
            count = -1;
        }

    #endif

    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1;

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count;
}

int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" words from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();

    #if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE)

        #if (ARDUINO < 100)

            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.send(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)(length * 2));

                bool msb = true;
                for (; Wire.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {

                        data[count] = Wire.receive() << 8;
                    } else {

                        data[count] |= Wire.receive();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }

                Wire.endTransmission();
            }
        #elif (ARDUINO == 100)

            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)(length * 2));

                bool msb = true;
                for (; Wire.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {

                        data[count] = Wire.read() << 8;
                    } else {

                        data[count] |= Wire.read();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }

                Wire.endTransmission();
            }
        #elif (ARDUINO > 100)

            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                Wire.beginTransmission(devAddr);
                Wire.write(regAddr);
                Wire.endTransmission();
                Wire.beginTransmission(devAddr);
                Wire.requestFrom(devAddr, (uint8_t)(length * 2));

                bool msb = true;
                for (; Wire.available() && count < length && (timeout == 0 || millis() - t1 < timeout);) {
                    if (msb) {

                        data[count] = Wire.read() << 8;
                    } else {

                        data[count] |= Wire.read();
                        #ifdef I2CDEV_SERIAL_DEBUG
                            Serial.print(data[count], HEX);
                            if (count + 1 < length) Serial.print(" ");
                        #endif
                        count++;
                    }
                    msb = !msb;
                }

                Wire.endTransmission();
            }
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        uint16_t intermediate[(uint8_t)length];
        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, (uint8_t *)intermediate, (uint8_t)(length * 2));
        if (status == 0) {
            count = length;
            for (uint8_t i = 0; i < length; i++) {
                data[i] = (intermediate[2*i] << 8) | intermediate[2*i + 1];
            }
        } else {
            count = -1;
        }

    #endif

    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1;

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count;
}

bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {

    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        b &= ~(mask);
        b |= data;
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {

    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        w &= ~(mask);
        w |= data;
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" bytes to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.beginTransmission(devAddr);
        Wire.send((uint8_t) regAddr);
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        Wire.beginTransmission(devAddr);
        Wire.write((uint8_t) regAddr);
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::beginTransmission(devAddr);
        Fastwire::write(regAddr);
    #endif
    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
            Wire.send((uint8_t) data[i]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
            Wire.write((uint8_t) data[i]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
            Fastwire::write((uint8_t) data[i]);
        #endif
    }
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        status = Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::stop();

    #endif
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status == 0;
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" words to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.beginTransmission(devAddr);
        Wire.send(regAddr);
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr);
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::beginTransmission(devAddr);
        Fastwire::write(regAddr);
    #endif
    for (uint8_t i = 0; i < length * 2; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
            Wire.send((uint8_t)(data[i] >> 8));
            Wire.send((uint8_t)data[i++]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
            Wire.write((uint8_t)(data[i] >> 8));
            Wire.write((uint8_t)data[i++]);
        #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
            Fastwire::write((uint8_t)(data[i] >> 8));
            status = Fastwire::write((uint8_t)data[i++]);
            if (status != 0) break;
        #endif
    }
    #if ((I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO < 100) || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE)
        Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE && ARDUINO >= 100)
        status = Wire.endTransmission();
    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)
        Fastwire::stop();

    #endif
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status == 0;
}

uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE

    boolean Fastwire::waitInt() {
        int l = 250;
        while (!(TWCR & (1 << TWINT)) && l-- > 0);
        return l > 0;
    }

    void Fastwire::setup(int khz, boolean pullup) {
        TWCR = 0;
        #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)

            if (pullup) PORTC |= ((1 << 4) | (1 << 5));
            else        PORTC &= ~((1 << 4) | (1 << 5));
        #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)

            if (pullup) PORTC |= ((1 << 0) | (1 << 1));
            else        PORTC &= ~((1 << 0) | (1 << 1));
        #else

            if (pullup) PORTD |= ((1 << 0) | (1 << 1));
            else        PORTD &= ~((1 << 0) | (1 << 1));
        #endif

        TWSR = 0;
        TWBR = ((16000L / khz) - 16) / 2;
        TWCR = 1 << TWEN;
    }

    byte Fastwire::beginTransmission(byte device) {
        byte twst, retry;
        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 1;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 2;

            TWDR = device << 1;
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 3;
            twst = TWSR & 0xF8;
        } while (twst == TW_MT_SLA_NACK && retry-- > 0);
        if (twst != TW_MT_SLA_ACK) return 4;
        return 0;
    }

    byte Fastwire::writeBuf(byte device, byte address, byte *data, byte num) {
        byte twst, retry;

        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 1;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 2;

            TWDR = device & 0xFE;
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 3;
            twst = TWSR & 0xF8;
        } while (twst == TW_MT_SLA_NACK && retry-- > 0);
        if (twst != TW_MT_SLA_ACK) return 4;

        TWDR = address;
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 5;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 6;

        for (byte i = 0; i < num; i++) {

            TWDR = data[i];
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 7;
            twst = TWSR & 0xF8;
            if (twst != TW_MT_DATA_ACK) return 8;
        }

        return 0;
    }

    byte Fastwire::write(byte value) {
        byte twst;

        TWDR = value;
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 1;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 2;
        return 0;
    }

    byte Fastwire::readBuf(byte device, byte address, byte *data, byte num) {
        byte twst, retry;

        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 16;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 17;

            TWDR = device & 0xfe;
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 18;
            twst = TWSR & 0xF8;
        } while (twst == TW_MT_SLA_NACK && retry-- > 0);
        if (twst != TW_MT_SLA_ACK) return 19;

        TWDR = address;
        TWCR = (1 << TWINT) | (1 << TWEN);
        if (!waitInt()) return 20;
        twst = TWSR & 0xF8;
        if (twst != TW_MT_DATA_ACK) return 21;

        retry = 2;
        do {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA);
            if (!waitInt()) return 22;
            twst = TWSR & 0xF8;
            if (twst != TW_START && twst != TW_REP_START) return 23;

            TWDR = device | 0x01;
            TWCR = (1 << TWINT) | (1 << TWEN);
            if (!waitInt()) return 24;
            twst = TWSR & 0xF8;
        } while (twst == TW_MR_SLA_NACK && retry-- > 0);
        if (twst != TW_MR_SLA_ACK) return 25;

        for (uint8_t i = 0; i < num; i++) {
            if (i == num - 1)
                TWCR = (1 << TWINT) | (1 << TWEN);
            else
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
            if (!waitInt()) return 26;
            twst = TWSR & 0xF8;
            if (twst != TW_MR_DATA_ACK && twst != TW_MR_DATA_NACK) return twst;
            data[i] = TWDR;

        }

        stop();

        return 0;
    }

    void Fastwire::reset() {
        TWCR = 0;
    }

    byte Fastwire::stop() {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        if (!waitInt()) return 1;
        return 0;
    }
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE

    uint8_t TwoWire::rxBuffer[NBWIRE_BUFFER_LENGTH];
    uint8_t TwoWire::rxBufferIndex = 0;
    uint8_t TwoWire::rxBufferLength = 0;

    uint8_t TwoWire::txAddress = 0;
    uint8_t TwoWire::txBuffer[NBWIRE_BUFFER_LENGTH];
    uint8_t TwoWire::txBufferIndex = 0;
    uint8_t TwoWire::txBufferLength = 0;

    void (*TwoWire::user_onRequest)(void);
    void (*TwoWire::user_onReceive)(int);

    static volatile uint8_t twi_transmitting;
    static volatile uint8_t twi_state;
    static uint8_t twi_slarw;
    static volatile uint8_t twi_error;
    static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
    static volatile uint8_t twi_masterBufferIndex;
    static uint8_t twi_masterBufferLength;
    static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
    static volatile uint8_t twi_rxBufferIndex;

    static volatile uint8_t twi_Return_Value;
    static volatile uint8_t twi_Done;
    void (*twi_cbendTransmissionDone)(int);
    void (*twi_cbreadFromDone)(int);

    void twi_init() {

        twi_state = TWI_READY;

        sbi(PORTC, 4);
        sbi(PORTC, 5);

        cbi(TWSR, TWPS0);
        cbi(TWSR, TWPS1);

        TWBR = ((CPU_FREQ / TWI_FREQ) - 16) / 2;

        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);

    }

    typedef struct {
        uint8_t address;
        uint8_t* data;
        uint8_t length;
        uint8_t wait;
        uint8_t i;
    } twi_Write_Vars;

    twi_Write_Vars *ptwv = 0;
    static void (*fNextInterruptFunction)(void) = 0;

    void twi_Finish(byte bRetVal) {
        if (ptwv) {
            free(ptwv);
            ptwv = 0;
        }
        twi_Done = 0xFF;
        twi_Return_Value = bRetVal;
        fNextInterruptFunction = 0;
    }

    uint8_t twii_WaitForDone(uint16_t timeout) {
        uint32_t endMillis = millis() + timeout;
        while (!twi_Done && (timeout == 0 || millis() < endMillis)) continue;
        return twi_Return_Value;
    }

    void twii_SetState(uint8_t ucState) {
        twi_state = ucState;
    }

    void twii_SetError(uint8_t ucError) {
        twi_error = ucError ;
    }

    void twii_InitBuffer(uint8_t ucPos, uint8_t ucLength) {
        twi_masterBufferIndex = 0;
        twi_masterBufferLength = ucLength;
    }

    void twii_CopyToBuf(uint8_t* pData, uint8_t ucLength) {
        uint8_t i;
        for (i = 0; i < ucLength; ++i) {
            twi_masterBuffer[i] = pData[i];
        }
    }

    void twii_CopyFromBuf(uint8_t *pData, uint8_t ucLength) {
        uint8_t i;
        for (i = 0; i < ucLength; ++i) {
            pData[i] = twi_masterBuffer[i];
        }
    }

    void twii_SetSlaRW(uint8_t ucSlaRW) {
        twi_slarw = ucSlaRW;
    }

    void twii_SetStart() {
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
    }

    void twi_write01() {
        if (TWI_MTX == twi_state) return;
        twi_transmitting = 0 ;
        if (twi_error == 0xFF)
            twi_Finish (0);
        else if (twi_error == TW_MT_SLA_NACK)
            twi_Finish (2);
        else if (twi_error == TW_MT_DATA_NACK)
            twi_Finish (3);
        else
            twi_Finish (4);
        if (twi_cbendTransmissionDone) return twi_cbendTransmissionDone(twi_Return_Value);
        return;
    }

    void twi_write00() {
        if (TWI_READY != twi_state) return;
        if (TWI_BUFFER_LENGTH < ptwv -> length) {
            twi_Finish(1);
            return;
        }
        twi_Done = 0x00;
        twii_SetState(TWI_MTX);
        twii_SetError(0xFF);
        twii_InitBuffer(0, ptwv -> length);
        twii_CopyToBuf(ptwv -> data, ptwv -> length);
        twii_SetSlaRW((ptwv -> address << 1) | TW_WRITE);
        twii_SetStart();
        fNextInterruptFunction = twi_write01;
        return twi_write01();
    }

    void twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait) {
        uint8_t i;
        ptwv = (twi_Write_Vars *)malloc(sizeof(twi_Write_Vars));
        ptwv -> address = address;
        ptwv -> data = data;
        ptwv -> length = length;
        ptwv -> wait = wait;
        fNextInterruptFunction = twi_write00;
        return twi_write00();
    }

    void twi_read01() {
        if (TWI_MRX == twi_state) return;
        if (twi_masterBufferIndex < ptwv -> length) ptwv -> length = twi_masterBufferIndex;
        twii_CopyFromBuf(ptwv -> data, ptwv -> length);
        twi_Finish(ptwv -> length);
        if (twi_cbreadFromDone) return twi_cbreadFromDone(twi_Return_Value);
        return;
    }

    void twi_read00() {
        if (TWI_READY != twi_state) return;
        if (TWI_BUFFER_LENGTH < ptwv -> length) twi_Finish(0);
        twi_Done = 0x00;
        twii_SetState(TWI_MRX);
        twii_SetError(0xFF);
        twii_InitBuffer(0, ptwv -> length - 1);
        twii_SetSlaRW((ptwv -> address << 1) | TW_READ);
        twii_SetStart();
        fNextInterruptFunction = twi_read01;
        return twi_read01();
    }

    void twi_readFrom(uint8_t address, uint8_t* data, uint8_t length) {
        uint8_t i;

        ptwv = (twi_Write_Vars *)malloc(sizeof(twi_Write_Vars));
        ptwv -> address = address;
        ptwv -> data = data;
        ptwv -> length = length;
        fNextInterruptFunction = twi_read00;
        return twi_read00();
    }

    void twi_reply(uint8_t ack) {

        if (ack){
            TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
        } else {
            TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
        }
    }

    void twi_stop(void) {

        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

        while (TWCR & _BV(TWSTO)) {
            continue;
        }

        twi_state = TWI_READY;
    }

    void twi_releaseBus(void) {

        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

        twi_state = TWI_READY;
    }

    SIGNAL(TWI_vect) {
        switch (TW_STATUS) {

            case TW_START:
            case TW_REP_START:

                TWDR = twi_slarw;
                twi_reply(1);
                break;

            case TW_MT_SLA_ACK:
            case TW_MT_DATA_ACK:

                if (twi_masterBufferIndex < twi_masterBufferLength) {

                    TWDR = twi_masterBuffer[twi_masterBufferIndex++];
                    twi_reply(1);
                } else {
                    twi_stop();
                }
                break;

            case TW_MT_SLA_NACK:
                twi_error = TW_MT_SLA_NACK;
                twi_stop();
                break;

            case TW_MT_DATA_NACK:
                twi_error = TW_MT_DATA_NACK;
                twi_stop();
                break;

            case TW_MT_ARB_LOST:
                twi_error = TW_MT_ARB_LOST;
                twi_releaseBus();
                break;

            case TW_MR_DATA_ACK:

                twi_masterBuffer[twi_masterBufferIndex++] = TWDR;

            case TW_MR_SLA_ACK:

                if (twi_masterBufferIndex < twi_masterBufferLength) {
                    twi_reply(1);
                } else {
                    twi_reply(0);
                }
                break;

            case TW_MR_DATA_NACK:

                twi_masterBuffer[twi_masterBufferIndex++] = TWDR;

            case TW_MR_SLA_NACK:
                twi_stop();
                break;

            case TW_NO_INFO:
                break;

            case TW_BUS_ERROR:
                twi_error = TW_BUS_ERROR;
                twi_stop();
                break;
        }

        if (fNextInterruptFunction) return fNextInterruptFunction();
    }

    TwoWire::TwoWire() { }

    void TwoWire::begin(void) {
        rxBufferIndex = 0;
        rxBufferLength = 0;

        txBufferIndex = 0;
        txBufferLength = 0;

        twi_init();
    }

    void TwoWire::beginTransmission(uint8_t address) {

        twi_transmitting = 1;

        txAddress = address;

        txBufferIndex = 0;
        txBufferLength = 0;
    }

    uint8_t TwoWire::endTransmission(uint16_t timeout) {

        twi_cbendTransmissionDone = NULL;
        twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
        int8_t ret = twii_WaitForDone(timeout);

        txBufferIndex = 0;
        txBufferLength = 0;

        return ret;
    }

    void TwoWire::nbendTransmission(void (*function)(int)) {
        twi_cbendTransmissionDone = function;
        twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
        return;
    }

    void TwoWire::send(uint8_t data) {
        if (twi_transmitting) {

            if (txBufferLength >= NBWIRE_BUFFER_LENGTH) {
                return;
            }

            txBuffer[txBufferIndex] = data;
            ++txBufferIndex;

            txBufferLength = txBufferIndex;
        } else {

        }
    }

    uint8_t TwoWire::receive(void) {

        uint8_t value = 0;

        if (rxBufferIndex < rxBufferLength) {
            value = rxBuffer[rxBufferIndex];
            ++rxBufferIndex;
        }

        return value;
    }

    uint8_t TwoWire::requestFrom(uint8_t address, int quantity, uint16_t timeout) {

        if (quantity > NBWIRE_BUFFER_LENGTH) {
            quantity = NBWIRE_BUFFER_LENGTH;
        }

        twi_cbreadFromDone = NULL;
        twi_readFrom(address, rxBuffer, quantity);
        uint8_t read = twii_WaitForDone(timeout);

        rxBufferIndex = 0;
        rxBufferLength = read;

        return read;
    }

    void TwoWire::nbrequestFrom(uint8_t address, int quantity, void (*function)(int)) {

        if (quantity > NBWIRE_BUFFER_LENGTH) {
            quantity = NBWIRE_BUFFER_LENGTH;
        }

        twi_cbreadFromDone = function;
        twi_readFrom(address, rxBuffer, quantity);

        rxBufferIndex = 0;
        rxBufferLength = quantity;

        return;
    }

    uint8_t TwoWire::available(void) {
        return rxBufferLength - rxBufferIndex;
    }

#endif
