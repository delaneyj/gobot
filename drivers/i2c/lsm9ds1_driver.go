package i2c

import (
	"fmt"

	"encoding/binary"

	"gobot.io/x/gobot"
)

const (
	lsm9Ds1MagnetometerAddress = 0x1C // Would be 0x1e if SDO_M is HIGH
	lsm9Ds1GyroAddress         = 0x6A // Would be 0x6b if SDO_AG is HIGH

	// // LSM9DS1 Accel/Gyro (XL/G) Registers
	lsm9Ds1ActThs       = 0x04
	lsm9Ds1ActDur       = 0x05
	lsm9Ds1IntGenCfgXl  = 0x06
	lsm9Ds1IntGenThsXXl = 0x07
	lsm9Ds1IntGenThsYXl = 0x08
	lsm9Ds1IntGenThsZXl = 0x09
	lsm9Ds1IntGenDurXl  = 0x0A
	lsm9Ds1ReferenceG   = 0x0B
	lsm9Ds1Int1Ctrl     = 0x0C
	lsm9Ds1Int2Ctrl     = 0x0D
	lsm9Ds1WhoAmIXg     = 0x0F
	lsm9Ds1CtrlReg1G    = 0x10
	lsm9Ds1CtrlReg2G    = 0x11
	lsm9Ds1CtrlReg3G    = 0x12
	lsm9Ds1OrientCfgG   = 0x13
	lsm9Ds1IntGenSrcG   = 0x14
	lsm9Ds1OutTempL     = 0x15
	lsm9Ds1OutTempH     = 0x16
	lsm9Ds1StatusReg_0  = 0x17
	lsm9Ds1OutXLG       = 0x18
	lsm9Ds1OutXHG       = 0x19
	lsm9Ds1OutYLG       = 0x1A
	lsm9Ds1OutYHG       = 0x1B
	lsm9Ds1OutZLG       = 0x1C
	lsm9Ds1OutZHG       = 0x1D
	lsm9Ds1CtrlReg4     = 0x1E
	lsm9Ds1CtrlReg5xl   = 0x1F
	lsm9Ds1CtrlReg6xl   = 0x20
	lsm9Ds1CtrlReg7Xl   = 0x21
	lsm9Ds1CtrlReg8     = 0x22
	lsm9Ds1CtrlReg9     = 0x23
	lsm9Ds1CtrlReg10    = 0x24
	lsm9Ds1IntGenSrcXl  = 0x26
	lsm9Ds1StatusReg_1  = 0x27
	lsm9Ds1OutXLXl      = 0x28
	lsm9Ds1OutXHXl      = 0x29
	lsm9Ds1OutYLXl      = 0x2A
	lsm9Ds1OutYHXl      = 0x2B
	lsm9Ds1OutZLXl      = 0x2C
	lsm9Ds1OutZHXl      = 0x2D
	lsm9Ds1FifoCtrl     = 0x2E
	lsm9Ds1FifoSrc      = 0x2F
	lsm9Ds1IntGenCfgG   = 0x30
	lsm9Ds1IntGenThsXhG = 0x31
	lsm9Ds1IntGenThsXlG = 0x32
	lsm9Ds1IntGenThsYhG = 0x33
	lsm9Ds1IntGenThsYlG = 0x34
	lsm9Ds1IntGenThsZhG = 0x35
	lsm9Ds1IntGenThsZlG = 0x36
	lsm9Ds1IntGenDurG   = 0x37

	// // LSM9DS1 Magneto Registers
	lsm9Ds1OffsetXRegLM = 0x05
	lsm9Ds1OffsetXRegHM = 0x06
	lsm9Ds1OffsetYRegLM = 0x07
	lsm9Ds1OffsetYRegHM = 0x08
	lsm9Ds1OffsetZRegLM = 0x09
	lsm9Ds1OffsetZRegHM = 0x0A
	lsm9Ds1WhoAmIM      = 0x0F
	lsm9Ds1CtrlReg1M    = 0x20
	lsm9Ds1CtrlReg2M    = 0x21
	lsm9Ds1CtrlReg3M    = 0x22
	lsm9Ds1CtrlReg4M    = 0x23
	lsm9Ds1CtrlReg5M    = 0x24
	lsm9Ds1StatusRegM   = 0x27
	lsm9Ds1OutXLM       = 0x28
	lsm9Ds1OutXHM       = 0x29
	lsm9Ds1OutYLM       = 0x2A
	lsm9Ds1OutYHM       = 0x2B
	lsm9Ds1OutZLM       = 0x2C
	lsm9Ds1OutZHM       = 0x2D
	lsm9Ds1IntCfgM      = 0x30
	lsm9Ds1IntSrcM      = 0x30
	lsm9Ds1IntThsLM     = 0x32
	lsm9Ds1IntThsHM     = 0x33

	// // LSM9DS1 WHO_AM_I Responses
	lsm9Ds1WhoAmIAgRsp = 0x68
	lsm9Ds1WhoAmIMRsp  = 0x3D

	// Gyroscope output data rate selection. Default value: 000
	// (Refer to Table 46 and Table 47)
	lsm9Ds1ORDPowerDownCutoffNA = 0x0 << 5
	lsm9Ds1ORD14_9              = 0x1 << 5
	lsm9Ds1ORD59_5              = 0x2 << 5
	lsm9Ds1ORD119               = 0x3 << 5
	lsm9Ds1ORD238               = 0x4 << 5
	lsm9Ds1ORD476               = 0x5 << 5
	lsm9Ds1ORD952               = 0x6 << 5
	lsm9Ds1ORDNA                = 0x7 << 5

	// Gryoscope contro register 4 flags
	lsm9Ds1Reg44dxl16D          = 0      //  interrupt generator uses 6D for position recognition
	lsm9Ds1Reg44dxl14D          = 1      //  interrupt generator uses 4D for position recognition
	lsm9Ds1Reg4lirxl1NotLatched = 0 << 1 //  interrupt request not latched
	lsm9Ds1Reg4lirxl1Latched    = 1 << 1 //  interrupt request latched
	lsm9Ds1Reg4xDisabled        = 0 << 3 // Gyroscope’s pitch axis (X) output disabled
	lsm9Ds1Reg4xEnabled         = 1 << 3 // Gyroscope’s pitch axis (X) output enabled
	lsm9Ds1Reg4yDisabled        = 0 << 4 // Gyroscope’s pitch axis (Y) output disabled
	lsm9Ds1Reg4yEnabled         = 1 << 4 // Gyroscope’s pitch axis (Y) output enabled
	lsm9Ds1Reg4zDisabled        = 0 << 5 // Gyroscope’s pitch axis (Z) output disabled
	lsm9Ds1Reg4zEnabled         = 1 << 5 // Gyroscope’s pitch axis (Z) output enabled

	// Gyroscope full-scale selection. Default value: 00 degrees per second
	// (00: 245 dps; 01: 500 dps; 10: Not Available; 11: 2000 dps)
	lsm9Ds1FSoffset = 3
	lsm9Ds1FS245    = 0x0 << lsm9Ds1FSoffset
	lsm9Ds1FS500    = 0x1 << lsm9Ds1FSoffset
	lsm9Ds1FSNA     = 0x2 << lsm9Ds1FSoffset
	lsm9Ds1FS2000   = 0x3 << lsm9Ds1FSoffset

	// Gyroscope bandwidth selection. Default value: 00
	// TODO Deal with cutoffs

	// Angular rate sensor sign and orientation register
	lsm9Ds1SignZPositive = 0 << 3 // Yaw axis (Z) angular rate sign positive
	lsm9Ds1SignZNegative = 1 << 3 // Yaw axis (Z) angular rate sign negative
	lsm9Ds1SignYPositive = 0 << 4 // Roll axis (Y) angular rate sign positive
	lsm9Ds1SignYNegative = 1 << 4 // Roll axis (Y) angular rate sign negative
	lsm9Ds1SignXPositive = 0 << 5 // Pitch axis (X) angular rate sign positive
	lsm9Ds1SignXNegative = 1 << 5 // Pitch axis (X) angular rate sign negative

	// Control Register 5. Linear acceleration sensor
	lsm9Ds1Reg5XAccDisabled = 0 << 3 // Accelerometer’s X-axis output disabled
	lsm9Ds1Reg5XAccEnabled  = 1 << 3 // Accelerometer’s X-axis output enable
	lsm9Ds1Reg5YAccDisabled = 0 << 4 // Accelerometer’s Y-axis output disabled
	lsm9Ds1Reg5YAccEnabled  = 1 << 4 // Accelerometer’s Y-axis output enable
	lsm9Ds1Reg5ZAccDisabled = 0 << 5 // Accelerometer’s Z-axis output disabled
	lsm9Ds1Reg5ZAccEnabled  = 1 << 5 // Accelerometer’s Z-axis output enable

	// Control Register 5. Decimation of acceleration data on OUT REG and FIFO. Default value: 00
	lsm9Ds1Reg5DecNone = 0 << 6 // no decimation (default)
	lsm9Ds1Reg5Dec2    = 1 << 6 // update every 2 samples
	lsm9Ds1Reg5Dec4    = 2 << 6 // update every 4 samples
	lsm9Ds1Reg5Dec8    = 3 << 6 // update every 8 samples

	// Control Register 6. Anti-aliasing filter bandwidth selection.
	lsm9Ds1Reg6bwxl408 = 0x0 << 0 // 408Hz
	lsm9Ds1Reg6bwxl211 = 0x1 << 0 // 215Hz
	lsm9Ds1Reg6bwxl105 = 0x2 << 0 // 105Hz
	lsm9Ds1Reg6bwxl50  = 0x3 << 0 // 50Hz

	// Control Register 6. Bandwidth selection
	lsm9Ds1Reg6ODRbased  = 0x0 << 2 // bandwidth determined by ODR selection: 408Hz when ODR=952Hz, 50Hz, 10 Hz; 211Hz when ODR=476Hz; 105Hz when ODR=238Hz; BW=50Hz when ODR=119Hz;
	lsm9Ds1Reg6BWXLbased = 0x1 << 2 // bandwidth selected according to BW_XL [2:1] selection)

	// Control Register 6. Accelerometer full-scale selection
	lsm9Ds1Reg6fsxl2g  = 0x0 << 3 // ±2g
	lsm9Ds1Reg6fsxl16g = 0x1 << 3 // ±16g
	lsm9Ds1Reg6fsxl4g  = 0x2 << 3 // ±4g
	lsm9Ds1Reg6fsxl8g  = 0x3 << 3 // ±8g

	// 	Control Register 6. Output data rate and power mode selection.
	lsm9Ds1Reg6ORDPowerDown = 0x0 << 5
	lsm9Ds1Reg6ORD10        = 0x1 << 5
	lsm9Ds1Reg6ORD50        = 0x2 << 5
	lsm9Ds1Reg6ORD119       = 0x3 << 5
	lsm9Ds1Reg6ORD238       = 0x4 << 5
	lsm9Ds1Reg6ORD476       = 0x5 << 5
	lsm9Ds1Reg6ORD952       = 0x6 << 5
	lsm9Ds1Reg6ORDna        = 0x7 << 5

	// Control Register 1M.
	lsm9Ds1Reg1MstDisabled       = 0x0 << 0 // Self-test disabled
	lsm9Ds1Reg1MstEnabled        = 0x1 << 0 // Self-test enabled
	lsm9Ds1Reg1MfastODRDisabled  = 0x0 << 1 // disable data rates higher than 80 Hz.
	lsm9Ds1Reg1MfastODREnabled   = 0x1 << 1 // enable data rates higher than 80 Hz.
	lsm9Ds1Reg1Mdo0_625          = 0x0 << 2 // Output data rate selection. 0.625Hz
	lsm9Ds1Reg1Mdo1_25           = 0x1 << 2 // Output data rate selection. 1.25Hz
	lsm9Ds1Reg1Mdo2_5            = 0x2 << 2 // Output data rate selection. 2.5Hz
	lsm9Ds1Reg1Mdo5              = 0x3 << 2 // Output data rate selection. 5Hz
	lsm9Ds1Reg1Mdo10             = 0x4 << 2 // Output data rate selection. 10Hz
	lsm9Ds1Reg1Mdo20             = 0x5 << 2 // Output data rate selection. 20Hz
	lsm9Ds1Reg1Mdo40             = 0x6 << 2 // Output data rate selection. 40Hz
	lsm9Ds1Reg1Mdo80             = 0x7 << 2 // Output data rate selection. 80Hz
	lsm9Ds1Reg1MomLow            = 0x0 << 5 // X and Y axes operative mode selection: low power mode
	lsm9Ds1Reg1MomMed            = 0x1 << 5 // X and Y axes operative mode selection: med power mode
	lsm9Ds1Reg1MomHigh           = 0x2 << 5 // X and Y axes operative mode selection: high power mode
	lsm9Ds1Reg1MomUltra          = 0x3 << 5 // X and Y axes operative mode selection: ultra power mode
	lsm9Ds1Reg1MtempCompDisabled = 0x0 << 7 // Temperature compensation disabled
	lsm9Ds1Reg1MtempCompEnabled  = 0x1 << 7 // Temperature compensation enabled

	// Control Register 2M.
	lsm9Ds1Reg2MsoftRstDefault      = 0x0 << 2 // Configuration registers and user register reset function default value
	lsm9Ds1Reg2MsoftRstOperation    = 0x1 << 2 // Configuration registers and user register reset function reset operation
	lsm9Ds1Reg2MrebootNormal        = 0x0 << 3 // Reboot memory content normal
	lsm9Ds1Reg2MrebootMemoryContent = 0x1 << 3 // Reboot memory content
	lsm9Ds1Reg2Mfs4                 = 0x0 << 5 //  Full-scale configuration. ± 4 gauss
	lsm9Ds1Reg2Mfs8                 = 0x1 << 5 //  Full-scale configuration. ± 8 gauss
	lsm9Ds1Reg2Mfs12                = 0x2 << 5 //  Full-scale configuration. ± 12 gauss
	lsm9Ds1Reg2Mfs16                = 0x3 << 5 //  Full-scale configuration. ± 16 gauss

	// Control Register 3M.
	lsm9Ds1Reg3MmdContinous  = 0x0 << 0 //  Operating mode selection. Continuous-conversion mode
	lsm9Ds1Reg3MmdSingle     = 0x1 << 0 //  Operating mode selection. Single-conversion mode
	lsm9Ds1Reg3MmdPowerDown  = 0x2 << 0 //  Operating mode selection. Power-down mode
	lsm9Ds1Reg3MsimWrite     = 0x0 << 2 // SPI Serial Interface mode selection. only write operations enabled
	lsm9Ds1Reg3MsimReadWrite = 0x1 << 2 // SPI Serial Interface mode selection.  read and write operations enable
	lsm9Ds1Reg3MlpDisabled   = 0x0 << 5 // Low-power mode configuration. the magnetic data rate is configured by the DO bits in the CTRL_REG1_M (0x20) register.
	lsm9Ds1Reg3MlpEnabled    = 0x1 << 5 // Low-power mode configuration.  the DO[2:0] is set to 0.625 Hz and the system performs, for each channel, the minimum number of averages.
	lsm9Ds1Reg3Mlp           = 0x1 << 2 // SPI Serial Interface mode selection.  read and write operations enable
	lsm9Ds1Reg3Mi2cEnabled   = 0x0 << 7 // Enable I2C interface.
	lsm9Ds1Reg3Mi2cDisable   = 0x1 << 7 // Disable I2C interface.

	// Control Register 4M.
	lsm9Ds1Reg4MbigEndian    = 0x0 << 1 // Big Endian data selection.
	lsm9Ds1Reg4MlittleEndian = 0x1 << 1 // Little Endian data selection.
	lsm9Ds1Reg4omzLow        = 0x0 << 2 // Z-axis operative mode selection. Low-power mode
	lsm9Ds1Reg4omzMed        = 0x1 << 2 // Z-axis operative mode selection. Medium-performance mode
	lsm9Ds1Reg4omzHigh       = 0x2 << 2 // Z-axis operative mode selection. High-performance mode
	lsm9Ds1Reg4omzUltra      = 0x3 << 2 // Z-axis operative mode selection. Ultra-high performance mode
)

// LSM9DS1Driver is the Gobot driver for the LSM9DS1 I2C 3D accelerometer, gyroscope, magnetometer device.
type LSM9DS1Driver struct {
	Config
	name                   string
	connector              Connector
	gyroAccConnection      Connection
	magnetometerConnection Connection
}

// NewLSM9DS1Driver creates a new driver for the LSM9DS1Driver I2C 9DOF device.
//
// Params:
//		conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//		i2c.WithBus(int):	bus to use with this driver
//		i2c.WithAddress(int):	address to use with this driver
//
func NewLSM9DS1Driver(a Connector, options ...func(Config)) *LSM9DS1Driver {
	d := &LSM9DS1Driver{
		name:      gobot.DefaultName("LSM9DS1"),
		connector: a,
		Config:    NewConfig(),
	}

	for _, option := range options {
		option(d)
	}

	// TODO: add commands to API
	return d
}

// Name returns the Name for the Driver
func (d *LSM9DS1Driver) Name() string { return d.name }

// SetName sets the Name for the Driver
func (d *LSM9DS1Driver) SetName(n string) { d.name = n }

// Connection returns the connection for the Driver
func (d *LSM9DS1Driver) Connection() gobot.Connection { return d.connector.(gobot.Connection) }

// Start initialized the 9DOF
func (d *LSM9DS1Driver) Start() error {
	var err error
	bus := d.GetBusOrDefault(d.connector.GetDefaultBus())

	gyroAddress := d.GetAddressOrDefault(lsm9Ds1GyroAddress)
	d.gyroAccConnection, err = d.connector.GetConnection(gyroAddress, bus)
	if err != nil {
		return err
	}

	magnetometerAddress := d.GetAddressOrDefault(lsm9Ds1MagnetometerAddress)
	d.magnetometerConnection, err = d.connector.GetConnection(magnetometerAddress, bus)
	if err != nil {
		return err
	}

	if err := d.validate(); err != nil {
		return err
	}

	if err := d.init(); err != nil {
		return err
	}

	return nil
}

func (d *LSM9DS1Driver) validate() error {
	whoXG, err := d.gyroAccConnection.ReadByteData(lsm9Ds1WhoAmIXg)
	if err != nil {
		return err
	}
	if whoXG != lsm9Ds1WhoAmIAgRsp {
		return fmt.Errorf("can't find gyro")
	}

	whoM, err := d.magnetometerConnection.ReadByteData(lsm9Ds1WhoAmIM)
	if err != nil {
		return err
	}
	if whoM != lsm9Ds1WhoAmIMRsp {
		return fmt.Errorf("can't find magnetometer")
	}

	return nil
}

func (d *LSM9DS1Driver) init() error {
	// initialise the gyroscope
	{
		// z, y, x axis enabled for gyro 0x38
		xyzEnabled := byte(lsm9Ds1Reg4xEnabled | lsm9Ds1Reg4yEnabled | lsm9Ds1Reg4zEnabled)
		if err := d.gyroAccConnection.WriteByteData(lsm9Ds1CtrlReg4, xyzEnabled); err != nil {
			return err
		}
		// Gyro ODR = 476Hz, 2000 dps
		if err := d.gyroAccConnection.WriteByteData(lsm9Ds1CtrlReg1G, lsm9Ds1ORD476|lsm9Ds1FS2000); err != nil {
			return err
		}
		// Swap orientation 0x38
		swapOrientation := byte(lsm9Ds1SignXNegative | lsm9Ds1SignYNegative | lsm9Ds1SignZNegative)
		if err := d.gyroAccConnection.WriteByteData(lsm9Ds1OrientCfgG, swapOrientation); err != nil {
			return err
		}
	}

	// initialise the accelerometer
	{
		// z, y, x axis enabled for accelerometer
		xyzEnabled := byte(lsm9Ds1Reg5XAccEnabled | lsm9Ds1Reg5YAccEnabled | lsm9Ds1Reg5ZAccEnabled)
		if err := d.gyroAccConnection.WriteByteData(lsm9Ds1CtrlReg5xl|lsm9Ds1Reg5DecNone, xyzEnabled); err != nil {
			return err
		}

		// +/- 16g output data rate 10Hz 0x28
		odr10g16 := byte(lsm9Ds1Reg6fsxl16g | lsm9Ds1Reg6ORD10)
		if err := d.gyroAccConnection.WriteByteData(lsm9Ds1CtrlReg6xl, odr10g16); err != nil {
			return err
		}
	}

	// initialise the magnetometer
	{
		// Temp compensation enabled,Low power mode mode,80Hz ODR  0x9c
		config := byte(lsm9Ds1Reg1MtempCompEnabled | lsm9Ds1Reg1MomLow | lsm9Ds1Reg1Mdo80)
		if err := d.magnetometerConnection.WriteByteData(lsm9Ds1CtrlReg1M, config); err != nil {
			return err
		}

		// +/-12gauss 0x40
		if err := d.magnetometerConnection.WriteByteData(lsm9Ds1CtrlReg2M, lsm9Ds1Reg2Mfs12); err != nil {
			return err
		}

		// continuos update 0x0
		if err := d.magnetometerConnection.WriteByteData(lsm9Ds1CtrlReg3M, lsm9Ds1Reg3MmdContinous); err != nil {
			return err
		}

		// lower power mode for Z axis 0x0
		if err := d.magnetometerConnection.WriteByteData(lsm9Ds1CtrlReg4M, lsm9Ds1Reg4omzLow); err != nil {
			return err
		}
	}

	return nil
}

// Halt returns true if devices is halted successfully
func (d *LSM9DS1Driver) Halt() (err error) { return }

func readInt16(c Connection, low, high uint8) (int16, error) {
	l, err := c.ReadByteData(low)
	if err != nil {
		return 0, err
	}
	h, err := c.ReadByteData(high)
	if err != nil {
		return 0, err
	}

	i := binary.PutVarint([]byte{l, h}, 0)
	return int16(i), nil
}

func (d *LSM9DS1Driver) AccelerationX() (int16, error) {
	return readInt16(d.gyroAccConnection, lsm9Ds1OutXLXl, lsm9Ds1OutXHXl)
}

func (d *LSM9DS1Driver) AccelerationY() (int16, error) {
	return readInt16(d.gyroAccConnection, lsm9Ds1OutYLXl, lsm9Ds1OutYHXl)
}

func (d *LSM9DS1Driver) AccelerationZ() (int16, error) {
	return readInt16(d.gyroAccConnection, lsm9Ds1OutZLXl, lsm9Ds1OutZHXl)
}

func (d *LSM9DS1Driver) MagnetometerX() (int16, error) {
	return readInt16(d.magnetometerConnection, lsm9Ds1OutXLM, lsm9Ds1OutXHM)
}

func (d *LSM9DS1Driver) MagnetometerY() (int16, error) {
	return readInt16(d.magnetometerConnection, lsm9Ds1OutYLM, lsm9Ds1OutYHM)
}

func (d *LSM9DS1Driver) MagnetometerZ() (int16, error) {
	return readInt16(d.magnetometerConnection, lsm9Ds1OutZLM, lsm9Ds1OutZHM)
}

func (d *LSM9DS1Driver) GyroX() (int16, error) {
	return readInt16(d.gyroAccConnection, lsm9Ds1OutXLG, lsm9Ds1OutXHG)
}

func (d *LSM9DS1Driver) GyroY() (int16, error) {
	return readInt16(d.gyroAccConnection, lsm9Ds1OutYLG, lsm9Ds1OutYHG)
}

func (d *LSM9DS1Driver) GyroZ() (int16, error) {
	return readInt16(d.gyroAccConnection, lsm9Ds1OutZLG, lsm9Ds1OutZHG)
}
