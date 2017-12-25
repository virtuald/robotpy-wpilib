
from . import data
hal_data = data.hal_data

from . import functions

class SPISimBase:
    '''
        Base class to use for SPI protocol simulators.
        
        Has all functions that need to be implemented, but throws exceptions
        when data is asked of it. Will throw away set* function data, as most
        low-fidelity simulation will probably not care about such things. 
    '''
    
    def initializeSPI(self, port, status):
        self.port = port
        status.value = 0
    
    def transactionSPI(self, port, dataToSend, dataReceived, size):
        '''
            To give data back use ``data_received``::
            
                data_received[:] = [1,2,3...]
            
            :returns: number of bytes returned
        '''
        raise NotImplementedError
    
    def writeSPI(self, port, dataToSend, sendSize):
        ''':returns: number of bytes written'''
        return sendSize
    
    def readSPI(self, port, buffer, count):
        '''
            To give data, do ``buffer[:] = [1,2,3...]``
            
            :returns: number of bytes read
        '''
        raise NotImplementedError
    
    def closeSPI(self, port):
        pass
    
    def setSPISpeed(self, port, speed):
        pass
    
    def setSPIOpts(self, port, msbFirst, sampleOnTrailing, clkIdleHigh):
        pass
    
    def setSPIChipSelectActiveHigh(self, port, status):
        pass
    
    def setSPIChipSelectActiveLow(self, port, status):
        pass
    
    def getSPIHandle(self, port):
        pass
    
    def setSPIHandle(self, port, handle):
        pass
        
    def initSPIAuto(self, port, bufferSize, status):
        status.value = 0

    def freeSPIAuto(self, port, status):
        status.value = 0

    def startSPIAutoRate(self, port, period, status):
        status.value = 0
        #self._autoPeriod = period
        #self._autoLastRead = functions.hooks.getTime()

    def startSPIAutoTrigger(self, port, digitalSourceHandle, analogTriggerType, triggerRising, triggerFalling, status):
        status.value = 0

    def stopSPIAuto(self, port, status):
        status.value = 0

    def setSPIAutoTransmitData(self, port, dataToSend, dataSize, zeroSize, status):
        status.value = 0

    def forceSPIAutoRead(self, port, status):
        status.value = 0

    def readSPIAutoReceivedData(self, port, buffer, numToRead, timeout, status):
        ''':returns: number of bytes read'''
        status.value = 0
        
        # just fill the buffer with a single value
        val = self._readSPIAutoReceivedData()
        valSz = len(val)
        
        if numToRead != 1:
            buffer[0:valSz] = val
        
        return valSz
        
        now = functions.hooks.getTime()
        samples = int((now - self._autoLastRead) / self._autoPeriod)
        if numToRead == 0:
            return samples
        
        self._autoLastRead = now
        
        off = 0
        for i in range(samples):
            ooff = off
            off += valSz
            buffer[ooff:off] = val
        
        return off
        
    # subclasses: implement this instead, returning a single value. The value
    #             will get scaled properly by the update rate
    def _readSPIAutoReceivedData(self) -> bytes:
        raise NotImplementedError

    def getSPIAutoDroppedCount(self, port, status):
        ''':returns: int32'''
        raise NotImplementedError
        


class ADXRS450_Gyro_Sim(SPISimBase):
    '''
        This returns the angle of the gyro as the value of::
            
            hal_data['robot']['adxrs450_spi_%d_angle']
            
        Where %d is the i2c port number. Angle should be in degrees.
    '''
    
    def __init__(self, gyro):
        self.kDegreePerSecondPerLSB = gyro.kDegreePerSecondPerLSB
        self.kSamplePeriod = gyro.kSamplePeriod
        
        self.maxSegments = 2048
        self.maxAnglePerSegment = 0x7fff * self.kSamplePeriod * self.kDegreePerSecondPerLSB
        
        self.lastAngle = 0
    
    def initializeSPI(self, port, status):
        self.angle_key = 'adxrs450_spi_%d_angle' % port
        self.rate_key = 'adxrs450_spi_%d_rate' % port
    
    def setSPIAutoTransmitData(self, port, data_to_send, sendSize, zeroSize, status):
        status.value = 0
    
    def readSPIAutoReceivedData(self, port, buffer, numToRead, timeout, status):
        ''':returns: number of bytes read'''
        status.value = 0
        
        current = hal_data['robot'].get(self.angle_key, 0)
        offset = current - self.lastAngle
        samples = 1 + int(offset / self.maxAnglePerSegment)
        samples = min(samples, self.maxSegments)
        
        # challenge: compute the amount of angle we're communicating without
        #            losing any..
        
        if numToRead != 0:
            # This is annoying, because we have to compute the offset from where we
            # are, then piece it up in up to 2048 chunks
            
            
            print("preoff",offset)
            offset = offset / (self.kSamplePeriod * self.kDegreePerSecondPerLSB)
            print('offset', offset, int(offset) & 0xffff, offset * 6.25e-06)
            
            data = 0x4000000 | ((int(offset) & 0xffff) << 10)
            #sprint('data %x %x %s' % (data, offset & 0xffff, self.kSamplePeriod * self.kDegreePerSecondPerLSB))
            
            buffer[0:4] = data.to_bytes(4, 'big')
        
        return samples * 4
    
    def readSPI(self, port, buffer, count):
        buffer[:] = (0xff000000 | (0x5200 << 5)).to_bytes(4, 'big')
        return count
