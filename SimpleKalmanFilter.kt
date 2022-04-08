/***************************************************************************************************
 *  measureVariance: Measurement Variance - How much do we expect to our measurement vary.
 *  processVariance: Process Variance - How fast your measurement moves.
 *  initialVariance: Estimation Uncertainty - Can be initialized with the same value as measurementVariance since the kalman filter will adjust its value.
 *
 *  Example:
 *  val kf: SimpleKalmanFilter = SimpleKalmanFilter(measurementVariance, processVariance, initialVariance)
 *
 *  while (1) {
 *    float x = analogRead(A0)
 *    float estimated_x = kf.getEstimate(x)
 *  }
 ***************************************************************************************************/

class SimpleKalmanFilter(var measureVariance: Double,
                         var processVariance: Double,
                         var initialEstimationVariance: Double = measureVariance)
{
  var gain: Double = NaN
    private set

  var estimationVariance = initialEstimationVariance
  private var lastMeasurement = NaN

  fun getEstimate(measurement: Double): Double
  {
    if (gain.isNaN())
      lastMeasurement = measurement

    gain = estimationVariance / (estimationVariance + measureVariance)

    val estimate = lastMeasurement + gain * (measurement - lastMeasurement)

    estimationVariance =
      (1 - gain) * estimationVariance + abs(lastMeasurement - estimate) * processVariance

    lastMeasurement = estimate

    return estimate

  }

  fun clear()
  {
    gain = NaN
    estimationVariance = initialEstimationVariance
    lastMeasurement = NaN

  }
}

