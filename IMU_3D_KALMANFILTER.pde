import processing.serial.*;
import processing.opengl.*;
import toxi.math.*;
import toxi.geom.*;
import toxi.processing.*;

Serial myPort;
float ax, ay, az, gx, gy, gz;
Quaternion quat = new Quaternion(1, 0, 0, 0);
ToxiclibsSupport gfx;

// Kalman Filter Class
class KalmanFilter {
  float qAngle = 0.001;  // Process noise covariance for the angle
  float qBias = 0.002;   // Process noise covariance for the bias
  float rMeasure = 0.02; // Measurement noise covariance

  float angle = 0;   // Filtered angle
  float bias = 0;    // Gyro bias
  float rate = 0;    // Gyroscope rate
  
  float P00 = 1, P01 = 0, P10 = 0, P11 = 1; // Error covariance matrix

  float update(float newAngle, float newRate, float dt) {
    // Prediction step
    rate = newRate - bias;
    angle += dt * rate;

    // Update covariance matrix
    P00 += dt * (dt * P11 - P01 - P10 + qAngle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += qBias * dt;

    // Compute Kalman gain
    float S = P00 + rMeasure;
    float K0 = P00 / S;
    float K1 = P10 / S;

    // Correction step
    float y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;

    // Update covariance matrix
    float P00_temp = P00;
    float P01_temp = P01;
    P00 -= K0 * P00_temp;
    P01 -= K0 * P01_temp;
    P10 -= K1 * P00_temp;
    P11 -= K1 * P01_temp;

    return angle;
  }
}

// Initialize Kalman filters for roll and pitch
KalmanFilter kalmanRoll = new KalmanFilter();
KalmanFilter kalmanPitch = new KalmanFilter();

void setup() {
  size(600, 600, P3D);
  gfx = new ToxiclibsSupport(this);
  myPort = new Serial(this, "COM4", 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(0);
  lights();
  translate(width / 2, height / 2);

  applyQuaternionRotation(quat);

  stroke(200);
  fill(150, 200, 0);
  box(200);
}

void serialEvent(Serial p) {
  String inData = p.readStringUntil('\n');
  if (inData != null) {
    try {
      JSONObject json = parseJSONObject(inData.trim()); // Parse incoming JSON
      if (json != null) {
        ax = json.getFloat("ax");
        ay = json.getFloat("ay");
        az = json.getFloat("az");
        gx = json.getFloat("gx");
        gy = json.getFloat("gy");
        gz = json.getFloat("gz");

        updateQuaternion();
      }
    } catch (Exception e) {
      println("Error parsing JSON: " + e);
      println("Received data: " + inData);
    }
  }
}

void updateQuaternion() {
  float dt = 0.01; // Time step (10 ms)

  // Compute angles from accelerometer (assuming flat start)
  float rollAcc = atan2(ay, az);
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az));

  // Apply Kalman Filter for roll and pitch
  float roll = kalmanRoll.update(rollAcc, radians(gx), dt);
  float pitch = kalmanPitch.update(pitchAcc, radians(gy), dt);

  // Convert back to quaternion
  quat = Quaternion.createFromEuler(roll, pitch, 0); // Assuming no yaw correction
  quat.normalize();
}

void applyQuaternionRotation(Quaternion q) {
  float qw = q.w;
  float qx = q.x;
  float qy = q.y;
  float qz = q.z;

  float[][] rotMatrix = {
    {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)},
    {2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)},
    {2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)}
  };

  applyMatrix(rotMatrix[0][0], rotMatrix[0][1], rotMatrix[0][2], 0,
              rotMatrix[1][0], rotMatrix[1][1], rotMatrix[1][2], 0,
              rotMatrix[2][0], rotMatrix[2][1], rotMatrix[2][2], 0,
              0, 0, 0, 1);
}
