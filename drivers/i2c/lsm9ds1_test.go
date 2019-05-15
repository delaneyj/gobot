package i2c

import (
	"errors"
	"testing"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/gobottest"
)

var _ gobot.Driver = (*LSM9DS1Driver)(nil)

// --------- HELPERS
func initTestLSM9DS1Driver() (driver *LSM9DS1Driver) {
	driver, _ = initTestLSM9DS1DriverWithStubbedAdaptor()
	return
}

func initTestLSM9DS1DriverWithStubbedAdaptor() (*LSM9DS1Driver, *i2cTestAdaptor) {
	adaptor := newI2cTestAdaptor()
	return NewLSM9DS1Driver(adaptor), adaptor
}

// --------- TESTS

func TestNewLSM9DS1Driver(t *testing.T) {
	// Does it return a pointer to an instance of LSM9DS1Driver?
	var bm interface{} = NewLSM9DS1Driver(newI2cTestAdaptor())
	_, ok := bm.(*LSM9DS1Driver)
	if !ok {
		t.Errorf("NewLSM9DS1Driver() should have returned a *LSM9DS1Driver")
	}

	b := NewLSM9DS1Driver(newI2cTestAdaptor())
	gobottest.Refute(t, b.Connection(), nil)
}

// Methods
func TestLSM9DS1DriverStart(t *testing.T) {
	hmc, _ := initTestLSM9DS1DriverWithStubbedAdaptor()

	gobottest.Assert(t, hmc.Start(), nil)
}

func TestLSM9DS1DriverStartConnectError(t *testing.T) {
	d, adaptor := initTestLSM9DS1DriverWithStubbedAdaptor()
	adaptor.Testi2cConnectErr(true)
	gobottest.Assert(t, d.Start(), errors.New("Invalid i2c connection"))
}

func TestLSM9DS1DriverHalt(t *testing.T) {
	hmc := initTestLSM9DS1Driver()

	gobottest.Assert(t, hmc.Halt(), nil)
}

func TestLSM9DS1DriverSetName(t *testing.T) {
	l := initTestLSM9DS1Driver()
	l.SetName("TESTME")
	gobottest.Assert(t, l.Name(), "TESTME")
}

func TestLSM9DS1DriverOptions(t *testing.T) {
	l := NewLSM9DS1Driver(newI2cTestAdaptor(), WithBus(2))
	gobottest.Assert(t, l.GetBusOrDefault(1), 2)
}
