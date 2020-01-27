use hal::blocking::i2c;
use hal::blocking::spi;
use hal::digital::v2::OutputPin;

use super::*;

#[cfg(feature = "std")]
use std::cell;

#[cfg(not(feature = "std"))]
use core::cell;

#[cfg(feature = "std")]
use std::marker;

#[cfg(not(feature = "std"))]
use core::marker;

/// A manager for managing a shared bus.
///
/// The manager owns the actual peripheral and hands out proxies which can be
/// used by your devices.
///
/// When creating a bus manager you need to specify which
/// mutex type should be used.
///
/// # Examples
/// ```
/// # use shared_bus;
/// # use shared_bus::BusManager;
/// # struct MyDevice;
/// # impl MyDevice {
/// #     pub fn new<T>(t: T) -> MyDevice { MyDevice }
/// # }
///
/// # let i2c = ();
/// // For example:
/// // let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 90.khz(), clocks, &mut rcc.apb1);
///
/// let manager = BusManager::<std::sync::Mutex<_>, _>::new(i2c);
///
/// // You can now acquire bus handles:
/// let mut handle1 = manager.acquire();
/// let mut mydevice = MyDevice::new(manager.acquire());
/// ```
pub struct BusManager<M: mutex::BusMutex<cell::RefCell<T>>, T>(M, marker::PhantomData<T>);

impl<M: mutex::BusMutex<cell::RefCell<T>>, T> BusManager<M, T> {
    /// Create a new manager for a bus peripheral `d`.
    ///
    /// When creating the manager you need to specify which mutex type should be used:
    /// ```
    /// # extern crate shared_bus;
    /// # use shared_bus::BusManager;
    /// # let bus = ();
    /// let manager = BusManager::<std::sync::Mutex<_>, _>::new(bus);
    /// ```
    pub fn new(d: T) -> BusManager<M, T> {
        let mutex = M::create(cell::RefCell::new(d));

        BusManager(mutex, marker::PhantomData)
    }

    /// Acquire a proxy for this bus.
    pub fn acquire_i2c<'a>(&'a self) -> BusProxy<'a, M, T, ()> {
        BusProxy(&self.0, marker::PhantomData, ())
    }

    /// Acquire a proxy for this bus.
    pub fn acquire_spi<'a, P>(&'a self, pin: P) -> BusProxy<'a, M, T, P> 
    where P: OutputPin 
    {
        BusProxy(&self.0, marker::PhantomData, pin)
    }
}

    

/// A proxy type that can be used instead of an actual bus peripheral.
///
/// `BusProxy` implements all bus traits and can thus be used in place of the
/// actual bus peripheral.
///
/// `BusProxies` are created by calling [`BusManager::acquire`]
pub struct BusProxy<'a, M: 'a + mutex::BusMutex<cell::RefCell<T>>, T, P>(
    &'a M,
    marker::PhantomData<T>,
    P,
);

impl<'a, M, I2C: i2c::Write> i2c::Write for BusProxy<'a, M, I2C, ()>
where
    M: 'a + mutex::BusMutex<cell::RefCell<I2C>>,
{
    type Error = I2C::Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.lock(|lock| {
            let mut i = lock.borrow_mut();
            i.write(addr, bytes)
        })
    }
}

impl<'a, M, I2C: i2c::Read> i2c::Read for BusProxy<'a, M, I2C, ()>
where
    M: 'a + mutex::BusMutex<cell::RefCell<I2C>>,
{
    type Error = I2C::Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.0.lock(|lock| {
            let mut i = lock.borrow_mut();
            i.read(address, buffer)
        })
    }
}

impl<'a, M, I2C: i2c::WriteRead> i2c::WriteRead for BusProxy<'a, M, I2C, ()>
where
    M: 'a + mutex::BusMutex<cell::RefCell<I2C>>,
{
    type Error = I2C::Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.lock(|lock| {
            let mut i = lock.borrow_mut();
            i.write_read(address, bytes, buffer)
        })
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum Error<S, P> {
    Spi(S),
    Pin(P),
}
pub trait ChipSelect {}

impl<'a, M, SPI: spi::Transfer<u8>, P> spi::Transfer<u8> for BusProxy<'a, M, SPI, P>
where
    M: 'a + mutex::BusMutex<cell::RefCell<SPI>>,
    P: OutputPin,
{
    type Error = Error<SPI::Error, P::Error>;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.0.lock(move |lock| {
            let mut i = lock.borrow_mut();

            i.transfer(words).map_err(Error::Spi)
        })
    }
}

impl<'a, M, SPI: spi::Write<u8>, P> spi::Write<u8> for BusProxy<'a, M, SPI, P>
where
    M: 'a + mutex::BusMutex<cell::RefCell<SPI>>,
    P: OutputPin,
{
    type Error = Error<SPI::Error, P::Error>;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.0.lock(|lock| {
            let mut i = lock.borrow_mut();

            i.write(words).map_err(Error::Spi)
        })
    }
}
