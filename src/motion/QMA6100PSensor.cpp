#include "QMA6100PSensor.h"

#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C

// Flag when an interrupt has been detected
volatile static bool QMA6100P_IRQ = false;

// Interrupt service routine
void QMA6100PSetInterrupt()
{
    QMA6100P_IRQ = true;
}

QMA6100PSensor::QMA6100PSensor(ScanI2C::FoundDevice foundDevice) : MotionSensor::MotionSensor(foundDevice) {}

bool QMA6100PSensor::init()
{
    // Initialise the sensor
    sensor = QMA6100PSingleton::GetInstance();
    if (!sensor->init(device))
        return false;

    // Enable simple Wake on Motion
    return sensor->setWakeOnMotion();
}

#ifdef QMA_6100P_INT_PIN

int32_t QMA6100PSensor::runOnce()
{
    // Wake on motion using hardware interrupts - this is the most efficient way to check for motion
    if (QMA6100P_IRQ) {
        QMA6100P_IRQ = false;
        sensor->clearInterrupts();
        wakeScreen();
    }
    return MOTION_SENSOR_CHECK_INTERVAL_MS;
}

#else

int32_t QMA6100PSensor::runOnce()
{
    // Wake on motion using polling  - this is not as efficient as using hardware interrupt pin (see above)
    auto status = sensor->setBank(0);
    if (sensor->status != QMA_6100P_Stat_Ok) {
        LOG_DEBUG("QMA6100PSensor::isWakeOnMotion failed to set bank - %s\n", sensor->statusString());
        return MOTION_SENSOR_CHECK_INTERVAL_MS;
    }

    QMA_6100P_INT_STATUS_t int_stat;
    status = sensor->read(AGB0_REG_INT_STATUS, (uint8_t *)&int_stat, sizeof(QMA_6100P_INT_STATUS_t));
    if (status != QMA_6100P_Stat_Ok) {
        LOG_DEBUG("QMA6100PSensor::isWakeOnMotion failed to read interrupts - %s\n", sensor->statusString());
        return MOTION_SENSOR_CHECK_INTERVAL_MS;
    }

    if (int_stat.WOM_INT != 0) {
        // Wake up!
        wakeScreen();
    }
    return MOTION_SENSOR_CHECK_INTERVAL_MS;
}

#endif

// ----------------------------------------------------------------------
// QMA6100PSingleton
// ----------------------------------------------------------------------

// Get a singleton wrapper for an Sparkfun QMA_6100P_I2C
QMA6100PSingleton *QMA6100PSingleton::GetInstance()
{
    if (pinstance == nullptr) {
        pinstance = new QMA6100PSingleton();
    }
    return pinstance;
}

QMA6100PSingleton::QMA6100PSingleton() {}

QMA6100PSingleton::~QMA6100PSingleton() {}

QMA6100PSingleton *QMA6100PSingleton::pinstance{nullptr};

// Initialise the QMA6100P Sensor
bool QMA6100PSingleton::init(ScanI2C::FoundDevice device)
{

// startup
#ifdef Wire1
    QMA_6100P_Status_e status =
        begin(device.address.port == ScanI2C::I2CPort::WIRE1 ? Wire1 : Wire, device.address.address == QMA6100P_ADDR ? 1 : 0);
#else
    //check chip id
    auto status = begin();
#endif
    if (status != true) {
        LOG_WARN("QMA6100PSensor::init begin failed\n" );
        return false;
    }
    delay(20);
    // SW reset to make sure the device starts in a known state
    if (softwareReset() != true) {
        LOG_WARN("QMA6100PSensor::init reset failed\n");
        return false;
    }
    delay(20);
    // Set range 
    if (!setRange(QMA_6100P_MPU_ACCEL_SCALE)) {
        LOG_WARN("QMA6100PSensor::init range failed");
        return false;
    }
    //set active mode
    if(!enableAccel()){
        LOG_WARN("ERROR :QMA6100PSensor::active mode set failed");
    }
    // set calibrateoffsets
    if(!calibrateOffsets()){
        LOG_WARN("ERROR :QMA6100PSensor:: calibration failed");
    }
#ifdef QMA_6100P_INT_PIN

    // Active low
    cfgIntActiveLow(true);
    LOG_DEBUG("QMA6100PSensor::init set cfgIntActiveLow - %s\n", statusString());

    // Push-pull
    cfgIntOpenDrain(false);
    LOG_DEBUG("QMA6100PSensor::init set cfgIntOpenDrain - %s\n", statusString());

    // If enabled, *ANY* read will clear the INT_STATUS register.
    cfgIntAnyReadToClear(true);
    LOG_DEBUG("QMA6100PSensor::init set cfgIntAnyReadToClear - %s\n", statusString());

    // Latch the interrupt until cleared
    cfgIntLatch(true);
    LOG_DEBUG("QMA6100PSensor::init set cfgIntLatch - %s\n", statusString());

    // Set up an interrupt pin with an internal pullup for active low
    pinMode(QMA_6100P_INT_PIN, INPUT_PULLUP);

    // Set up an interrupt service routine
    attachInterrupt(QMA_6100P_INT_PIN, QMA6100PSetInterrupt, FALLING);

#endif
    return true;
}

#ifdef QMA_6100P_DMP_IS_ENABLED

// Stub
bool QMA6100PSensor::initDMP()
{
    return false;
}

#endif

bool QMA6100PSingleton::setWakeOnMotion()
{
    // Set WoM threshold in milli G's
    auto status = WOMThreshold(QMA_6100P_WOM_THRESHOLD);
    if (status != QMA_6100P_Stat_Ok)
        return false;

    // Enable WoM Logic mode 1 = Compare the current sample with the previous sample
    status = WOMLogic(true, 1);
    LOG_DEBUG("QMA6100PSensor::init set WOMLogic - %s\n", statusString());
    if (status != QMA_6100P_Stat_Ok)
        return false;

    // Enable interrupts on WakeOnMotion
    status = intEnableWOM(true);
    LOG_DEBUG("QMA6100PSensor::init set intEnableWOM - %s\n", statusString());
    return status == QMA_6100P_Stat_Ok;

    // Clear any current interrupts
    QMA6100P_IRQ = false;
    clearInterrupts();
    return true;
}

#endif