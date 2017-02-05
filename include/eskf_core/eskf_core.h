#ifndef ESKF_CORE
#define ESKF_CORE

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Imu.h>
#include <chrono> // [DEBUG] For tic-toc computing
#include <thread>
#include <queue>

// Eigen
#include <Eigen/Dense>

// Internal Libs
#include "eskf_core/state.h"
#include "eskf_core/eskf_utility.h"
#include "vo_core/vo_state.h"
#include "eskf_core/eskf_container.h"

using namespace Eigen;
using namespace std;
using boost::shared_ptr;

class ESKF_Core
{
  public:
    ESKF_Core();
    ~ESKF_Core();

    // For algorithn verification (by MSFEKF)
    //EKF_State test_new;
    //EKF_State test_old;

    //void EKF_Prop(const ros::TimerEvent&);
    bool StartProcessing(Quaterniond &Cam2Imu_q, Vector3d &Cam2Imu_t, bool showInfo = false);
    bool MultiThreadUpdate(VOTRack &VoTrack);

  private:
    ros::NodeHandle n;
    ros::Subscriber Imu_sub;

    queue<VOTRack> MeasurementsQueue;
    ESKF_Container<ESKF_State> StateBuffer;

    ESKF_UTIL eskfUtil; // For useful constants and functions
    VOTRack VoTrack;

    thread update_thread;

    void Initialize(ESKF_Container<ESKF_State> &StateBuffer, queue<VOTRack> &MeasurementsQueue, const double &initTimeStamp, ESKF_UTIL &eskfUtil, bool UsingPrevState = false);
    void ESKF_Prop(boost::shared_ptr<ESKF_State> &state_new, boost::shared_ptr<ESKF_State> &state_old, ESKF_UTIL &eskfUtil);
    void ESKF_Update(VOTRack &VoTrack);
    void QueuingMeasHandler(queue<VOTRack> &MeasurementsQueue);
    void IMU_CB(const sensor_msgs::Imu ImuMsg);
    void StateTFPublish(boost::shared_ptr<ESKF_State> &state_new, ESKF_UTIL &eskfUtil);

};

ESKF_Core::ESKF_Core()
{
  // Declare Callback and Timer Funcs
  Imu_sub = n.subscribe(this->eskfUtil.ImuSubTopic, 20, &ESKF_Core::IMU_CB, this);

  this->eskfUtil.Trigger_id = false; // Trigger signal from VO
  this->eskfUtil.INIT_id = false; // flag for the state of initialization
}

ESKF_Core::~ESKF_Core()
{
  cout << "\nWait for the ESKF update thread finished ..." << endl;
  this->StateBuffer.Clear();
  this->update_thread.join();
  this->MeasurementsQueue.empty();
}

void ESKF_Core::StateTFPublish(boost::shared_ptr<ESKF_State> &state_new, ESKF_UTIL &eskfUtil)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //tf::poseEigenToTF(this->voTrack.Global_Track_A, transform);
  tf::Quaternion q;
  tf::Vector3 vec;
  tf::quaternionEigenToTF(eskfUtil.Vec2Quad(state_new->q), q);
  tf::vectorEigenToTF(state_new->p, vec);
  transform.setOrigin(vec);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), eskfUtil.TF_parent, eskfUtil.TF_child));

}

bool ESKF_Core::StartProcessing(Quaterniond &Cam2Imu_q, Vector3d &Cam2Imu_t, bool showInfo)
{
  /*if(Cam2Imu_t.sum() == 0 && Cam2Imu_q.vec().sum() == 0)
  {
    cout << "\n--- The matrix of Cam2Imu has error definition ---" << endl;
    return false;
  }
  */
  this->eskfUtil.Cam2Imu_q = Cam2Imu_q;
  this->eskfUtil.Cam2Imu_t = Cam2Imu_t;
  this->eskfUtil.Trigger_id = true;
  if(showInfo)
  {
    cout << "\nCamera to Imu Extrinsics matrix found" << endl;
    cout << "Cam to Imu q: " << Cam2Imu_q.w() << ", " << Cam2Imu_q.vec().transpose() << endl;
    cout << "Cam to Imu t: " << Cam2Imu_t.transpose() << endl;
  }

  cout << "\n --- ESKF start processing ---" << endl;
  return true;
}

void ESKF_Core::Initialize(ESKF_Container<ESKF_State> &StateBuffer, queue<VOTRack> &MeasurementsQueue, const double &initTimeStamp, ESKF_UTIL &eskfUtil, bool UsingPrevState)
{
  boost::shared_ptr<ESKF_State> state_init(new ESKF_State);

  if(UsingPrevState == true && StateBuffer.Size()>0)
  {
    state_init = StateBuffer.GetLast();
    state_init->timestamp = initTimeStamp;
    state_init->sequence = state_init->sequence + 1;
    StateBuffer.Insert(state_init);
    cout << endl << "ReInitialize down (using previous state)!!" << endl;
    eskfUtil.INIT_id = true;
    return;
  }
  // State: p,v, q, ba, bw
  StateBuffer.Clear();
  state_init->q << 1.0, 0.0, 0.0, 0.0;
  state_init->p << 0.0, 0.0, 0.0;
  state_init->v << 0.0, 0.0, 0.0;
  state_init->ba << 0.0, 0.0, 0.0;
  state_init->bw << 0.0, 0.0, 0.0;
  state_init->P = MatrixXd::Identity(15, 15) * eskfUtil.initial_process_covariance;

  state_init->am << 0.0, 0.0, 0.0;
  state_init->wm << 0.0, 0.0, 0.0;

  state_init->timestamp = initTimeStamp;
  state_init->sequence = 0;

  eskfUtil.Fd = MatrixXd::Identity(15, 15);
  eskfUtil.Qd = MatrixXd::Identity(12, 12);
  eskfUtil.Gc = MatrixXd::Zero(15, 12);
  eskfUtil.Gc.block<12,12>(3,0) = MatrixXd::Identity(12, 12); //Block size (p,q), starting at (i,j)

  StateBuffer.Insert(state_init);
  MeasurementsQueue.empty();

  /* // For algorithn verification (by MSFEKF)
  this->test_new = this->state_new;
  this->test_old = this->state_old;
  this->test_new.q << 0.0, 0.0, 0.0, 1.0;
  this->test_old.q << 0.0, 0.0, 0.0, 1.0;
  */
  cout.precision(17);
  cout << endl << "=== Initial State ====" << endl;
  cout << "Time: " << initTimeStamp << endl;
  cout << "p: " << endl << state_init->p.transpose() << endl;
  cout << "v: " << endl << state_init->v.transpose() << endl;
  cout << "q: " << endl << state_init->q.transpose() << endl;
  cout.precision(5);
  cout << "P: " << endl << state_init->P << endl;
  cout << "Fd: " << endl << eskfUtil.Fd << endl;
  cout << "Gc: " << endl << eskfUtil.Gc << endl;
  cout << "Qd: " << endl << eskfUtil.Qd << endl;
  cout << "H: " << endl << eskfUtil.H << endl;

  cout << endl << "Initialize down !!" << endl;
  eskfUtil.INIT_id = true;
  return;
}

void ESKF_Core::IMU_CB(const sensor_msgs::Imu ImuMsg)
{
  if(this->eskfUtil.Trigger_id == false)
    return;

  // For the first loop initialization
  if(this->StateBuffer.Size() == 0 || this->eskfUtil.INIT_id == false)
  {
    Initialize(this->StateBuffer, this->MeasurementsQueue, ImuMsg.header.stamp.toSec(), this->eskfUtil);
    return;
  }

  boost::shared_ptr<ESKF_State> state_new(new ESKF_State);
  state_new->am << ImuMsg.linear_acceleration.x, ImuMsg.linear_acceleration.y, ImuMsg.linear_acceleration.z;
  state_new->wm << ImuMsg.angular_velocity.x, ImuMsg.angular_velocity.y, ImuMsg.angular_velocity.z;
  state_new->sequence = ImuMsg.header.seq;
  state_new->timestamp = ImuMsg.header.stamp.toSec();

  // ============================
  // ===== ESKF Propagation =====
  // ============================
  // --- Set tic time ---
  auto t0 = std::chrono::high_resolution_clock::now();

  boost::shared_ptr<ESKF_State> state_old =  this->StateBuffer.GetLast();
  const double dt = state_new->timestamp - state_old->timestamp;

  //## Step0: Checking all values are in the reasonable range before propagating
  if(dt < 0)
  {
    cout << endl << "dt is negative ..." << endl;
    cout << "Reinitialize !" << endl;
    Initialize(this->StateBuffer, this->MeasurementsQueue, state_new->timestamp, this->eskfUtil);
  }

  if(dt >= this->eskfUtil.imuReadingTimeMax)
  {
    cout << endl <<  "[Warning] The interval between readings is too big !!" << endl;
    cout << "dt is: " << dt << endl;
    cout << "Reinitialize !" << endl;
    Initialize(this->StateBuffer, this->MeasurementsQueue, state_new->timestamp, this->eskfUtil);
    return;
  }

  if(state_new->am.norm() >= this->eskfUtil.imuReadingAccelMax )
  {
    cout << endl <<  "[Warning] The accelerometer readings is too large, am is: " << state_new->am.transpose() << endl;
    cout << "Take previous am reading !" << endl;
    state_new->am = state_old->am;
  }

  if(state_new->wm.norm() >= this->eskfUtil.imuReadingGyroMax )
  {
    cout << endl <<  "[Warning] The gyroscope readings is too large, wm is: " << state_new->wm.transpose() << endl;
    cout << "Take previous wm reading !" << endl;
    state_new->wm = state_old->wm;
  }
  // ===== ESKF_Prop Func =====
  ESKF_Prop(state_new, state_old, this->eskfUtil);

  // Put current new state into StateContainer
  this->StateBuffer.Insert(state_new);

  // Keep the size of StateContainer
  this->StateBuffer.ClearOlderThan(this->eskfUtil.stateBufferSizeInSec);

  // Try to handle the measurements in the queue
  QueuingMeasHandler(this->MeasurementsQueue);

  // --- Count the computation time ---
  auto t1_temp = std::chrono::high_resolution_clock::now(); // Set toc time

  // Broadcasting TF
  StateTFPublish(state_new, this->eskfUtil);

  if(this->eskfUtil.SHOW_Info)
  {
    cout << endl << "=== State ====" << endl;
    cout.precision(17);
    cout << "timestamp: "  << fixed << this->StateBuffer.GetLast()->timestamp << endl;
    cout << "sequence: "  << this->StateBuffer.GetLast()->sequence << endl;
    cout << "p: " << endl << this->StateBuffer.GetLast()->p.transpose() << endl;
    cout << "v: " << endl << this->StateBuffer.GetLast()->v.transpose() << endl;
    cout << "q: " << endl << this->StateBuffer.GetLast()->q.transpose() << endl;
    cout.precision(5);
    if(this->eskfUtil.DEBUG_Mode)
      cout << "P: " << endl << this->StateBuffer.GetLast()->P << endl;
    cout << "Processing Time: " <<  1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1_temp-t0).count() << endl;
    cout << "Processing Hz: " <<  1.0/(1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1_temp-t0).count()) << endl;
  }
}

void ESKF_Core::ESKF_Prop(boost::shared_ptr<ESKF_State> &state_new, boost::shared_ptr<ESKF_State> &state_old, ESKF_UTIL &eskfUtil)
{
  const double dt = state_new->timestamp - state_old->timestamp;

  //## Step1: Propagate the bias (under guassian assumption, they should be constant)
  state_new->ba = state_old->ba;
  state_new->bw = state_old->bw;

  //## Step2: Estimate the a and w by IMU reading
  const Vector3d a_hat_new = state_new->am - state_new->ba;
  const Vector3d a_hat_old = state_old->am - state_old->ba;
  const Vector3d w_hat_new = state_new->wm - state_new->bw;
  const Vector3d w_hat_old = state_old->wm - state_old->bw;
  const Vector4d w_new(0.0, w_hat_new[0], w_hat_new[1], w_hat_new[2] );
  const Vector4d w_old(0.0, w_hat_old[0], w_hat_old[1], w_hat_old[2] );
  const Matrix4d Omega_old = eskfUtil.QuatMat_L(w_old);
  const Matrix4d Omega_new = eskfUtil.QuatMat_L(w_new);
  Vector4d w_ave = (w_new + w_old) / 2.0;

  //## Step3: Propagate q v and p
  // For q - First order quaternion integration (reference to MSF-EKF)
  // We used Hamilton definition for quaternion: q
  int div = 1;
  Vector4d Exp_w(1.0, 0.0, 0.0, 0.0);
  w_ave = w_ave * 0.5 * dt;

  for (int i = 1; i < 4; i++) {  // Can be made fourth order or less to save cycles.
    div *= i;
    Exp_w = Exp_w + w_ave / div;
    w_ave = eskfUtil.QuatMat_L(w_ave) * w_ave;
  }

  const Matrix4d quatMat_old = eskfUtil.QuatMat_L(state_old->q);
  state_new->q = quatMat_old*( Exp_w + 1.0/48.0*(Omega_old*w_new - Omega_new*w_old)*dt*dt );
  eskfUtil.QuatNormal(state_new->q); // Normalize

  // For v
  Vector3d tmp_v;
  tmp_v = ( eskfUtil.Quat2Rot(state_new->q)*a_hat_new + eskfUtil.Quat2Rot(state_old->q)*a_hat_old )/2.0; // Get the average value between the old and new state
  state_new->v = state_old->v + (tmp_v - eskfUtil.gravity)*dt;

  // for p
  state_new->p = state_old->p + (state_old->v + state_new->v)/2.0*dt;

  /*  //## [DEBUG] Verified with MSFEKF algorithm ##
   *   Matrix<double, 3, 1> dv;
  const Vector3d ew = w_hat_new;
  const Vector3d ewold = w_hat_old;
  const Vector3d ea = a_hat_new;
  const Vector3d eaold = a_hat_old;
  const Matrix4d Omega = eskfUtil.OmegaMatJPL(ew);
  const Matrix4d OmegaOld = eskfUtil.OmegaMatJPL(ewold);
  Matrix4d OmegaMean = eskfUtil.OmegaMatJPL((ew + ewold) / 2);

  // First order quaternion integration, this is kind of costly and may not add
  // a lot to the quality of propagation...
  div = 1;
  Matrix4d MatExp;
  MatExp.setIdentity();
  OmegaMean *= 0.5 * dt;
  for (int i = 1; i < 5; i++) {  // Can be made fourth order or less to save cycles.
    div *= i;
    MatExp = MatExp + OmegaMean / div;
    OmegaMean *= OmegaMean;
  }

  // First oder quat integration matrix.
  const Matrix4d quat_int = MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;

  // First oder quaternion integration. ( q in the order x y z w )
  this->test_new.q = quat_int * this->test_old.q;
  eskfUtil.QuatNormal(this->test_new.q);

  dv = ( eskfUtil.Quat2RotJPL(this->test_new.q) * ea + eskfUtil.Quat2RotJPL(this->test_old.q) * eaold) / 2;
  this->test_new.v = this->test_old.v + (dv - eskfUtil.gravity) * dt;
  this->test_new.p = this->test_old.p + ( (this->test_new.v + this->test_old.v ) / 2 * dt);
  cout << endl << "=== MSFEKF ====" << endl;
  cout << "p: " << endl << this->test_new.p.transpose() << endl;
  cout << "v: " << endl << this->test_new.v.transpose() << endl;
  cout << "q: " << endl << this->test_new.q.transpose() << endl;
  */

  //## Step4: Propagate state covariance matrix
  // State: p, v, q, ba, bw
  //
  // Key equation:
  // P_new = Fd*P_old*Fd' + Gc*Qd*Gc'
  //
  // Fd was computed based on MSF-EKF (ETHZ) method which was reference to:
  // Stephan Weiss and Roland Siegwart.
  // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
  // IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
  //
  // Fi and Q matrix adopt the equation listed in:
  // Quaternion kinematics for the error-state KF, Joan Sola, 2016
  // (And also: Stochastic Models, Estimation and Control, p.171)

  // STD of noise sources (all in continuous state)
  const double na  = eskfUtil.accelerometer_noise_density;
  const double nba = eskfUtil.accelerometer_random_walk;
  const double nw  = eskfUtil.gyroscope_noise_density;
  const double nbw = eskfUtil.gyroscope_random_walk;

  // Bias corrected IMU readings.
  const Matrix3d a_sk = eskfUtil.Skew(a_hat_new); // [a_hat]x matrix
  const Matrix3d w_sk = eskfUtil.Skew(w_hat_new); // [w_hat]x matrix
  const Matrix3d eye3 = MatrixXd::Identity(3, 3);
  const Matrix3d C_eq = eskfUtil.Quat2Rot(state_new->q);

  // Construct the matrix Fd
  const double dt_p2_2 = dt * dt * 0.5;
  const double dt_p3_6 = dt_p2_2 * dt / 3.0;
  const double dt_p4_24 = dt_p3_6 * dt * 0.25;
  const double dt_p5_120 = dt_p4_24 * dt * 0.2;

  const Matrix3d Ca3 = C_eq * a_sk;
  const Matrix3d A = Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
  const Matrix3d B = Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
  const Matrix3d D = -A;
  const Matrix3d E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
  const Matrix3d F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
  const Matrix3d C = Ca3 * F;

  eskfUtil.Fd.block<3,3>(0,3)  = dt * eye3;
  eskfUtil.Fd.block<3,3>(0,6)  = A;
  eskfUtil.Fd.block<3,3>(0,9)  = -C_eq * dt_p2_2;
  eskfUtil.Fd.block<3,3>(0,12) = B;

  eskfUtil.Fd.block<3,3>(3,6)  = C;
  eskfUtil.Fd.block<3,3>(3,9)  = -C_eq * dt;
  eskfUtil.Fd.block<3,3>(3,12) = D;

  eskfUtil.Fd.block<3,3>(6,6)  = E;
  eskfUtil.Fd.block<3,3>(6,12) = F;

  // Construct the matrix Qd
  eskfUtil.Qd.block<3,3>(0,0) = nba*nba * dt*dt * eye3;
  eskfUtil.Qd.block<3,3>(3,3) = nbw*nbw * dt*dt * eye3;
  eskfUtil.Qd.block<3,3>(6,6) = na*na * dt * eye3;
  eskfUtil.Qd.block<3,3>(9,9) = nw*nw * dt * eye3;

  // Propagate the covariance matrix P_new = Fd*P_old*Fd' + Gc*Qd*Gc'
  state_new->P = eskfUtil.Fd * state_old->P * eskfUtil.Fd.transpose() + eskfUtil.Gc * eskfUtil.Qd * eskfUtil.Gc.transpose();
}

// function for obtaining external VoTrack obj generated by VO part
bool ESKF_Core::MultiThreadUpdate(VOTRack &myVoTrack)
{
  /*
  if(this->update_thread.joinable() == false)
  {
    this->VoTrack = myVoTrack;
    this->update_thread = thread(&ESKF_Core::ESKF_Update, this);
    return true;
  }
  else
    return false;*/

  ESKF_Update(myVoTrack);
  return true;
}

void ESKF_Core::QueuingMeasHandler(queue<VOTRack> &MeasurementsQueue)
{
  if (MeasurementsQueue.empty())
    return;

  VOTRack VoTrack = MeasurementsQueue.front();
  MeasurementsQueue.pop();
  ESKF_Update(VoTrack);
}

void ESKF_Core::ESKF_Update(VOTRack &VoTrack)
{
  if(this->eskfUtil.DEBUG_Mode)
    cout << "\n=== Measurement Update Processing ===" << endl;
  // --- Set tic time ---
  auto t0 = std::chrono::high_resolution_clock::now();

  const double measTime_keyframe = VoTrack.timeStamp_keyframe;
  const double measTime_previous = VoTrack.timeStamp_prev;
  const double measTime_current  = VoTrack.timeStamp_curr;

  const int statebuffersize = this->StateBuffer.Size();
  if(statebuffersize <= 2)
  {
    if(this->eskfUtil.SHOW_Info)
      cout << "\n[Update] State buffer size is: " << statebuffersize << endl;
    return;
  }

  const double lastStateTime = this->StateBuffer.GetLast()->timestamp;
  if( lastStateTime < measTime_current)
  {
    if(this->eskfUtil.SHOW_Info)
    {
      cout.precision(17);
      cout << "\n[Update] Measurement is newer than the latest state: " << endl;
      if(this->eskfUtil.DEBUG_Mode)
      {
        cout << "the last state time: " << lastStateTime << endl;
        cout << "the last measure time: " << measTime_current << endl;
        cout << "Push measurement into queue" << endl;
      }
    }
    this->MeasurementsQueue.push(VoTrack);
    return;
  }

  if(this->StateBuffer.GetFirst()->timestamp > measTime_keyframe)
  {
    if(this->eskfUtil.SHOW_Info)
      cout << "\n[Update] Keyframe time is older than the first state: " << endl;
    return;
  }

  if(measTime_current-measTime_previous >= this->eskfUtil.updateTimeMax)
    return;
  if(measTime_current-measTime_keyframe <= 0)
    return;
  if(measTime_current == 0 || measTime_keyframe == 0 || measTime_previous == 0)
    return;

  boost::shared_ptr<ESKF_State> StateAtKeyframe = this->StateBuffer.GetClosest(measTime_keyframe);
  boost::shared_ptr<ESKF_State> StateAtUpdate   = this->StateBuffer.GetClosest(measTime_current);
  const Quaterniond StateAtKeyframe_q = this->eskfUtil.Vec2Quad(StateAtKeyframe->q); // Since q in state is vector4d type
  const Quaterniond StateAtUpdate_q = this->eskfUtil.Vec2Quad(StateAtUpdate->q); // Since q in state is vector4d type

  Quaterniond odomInIMU_q = this->eskfUtil.Cam2Imu_q*VoTrack.odom_q;
  odomInIMU_q.normalize();
  Vector3d odomInIMU_t = this->eskfUtil.Cam2Imu_q._transformVector(VoTrack.odom_t) + this->eskfUtil.Cam2Imu_t;

  Quaterniond measUpdate_q = StateAtKeyframe_q * odomInIMU_q;
  measUpdate_q.normalize();
  Vector3d measUpdate_p = StateAtKeyframe_q._transformVector(odomInIMU_t) + StateAtKeyframe->p;

  const Matrix3d eye_3 = MatrixXd::Identity(3, 3);
  const Matrix<double, 6, 6> eye_6 = MatrixXd::Identity(6, 6);
  const Matrix<double, 15, 15> eye_15 = MatrixXd::Identity(15, 15);
  //if(this->eskfUtil.UsingFixedVOCovariance)
    const double measureNoiseSTD_q = this->eskfUtil.vo_fixedstd_q;
    const double measureNoiseSTD_p = this->eskfUtil.vo_fixedstd_p;
  Matrix<double, 6, 6> R = eye_6;
  R.block<3,3>(0,0) = measureNoiseSTD_p * measureNoiseSTD_p * eye_3;
  R.block<3,3>(3,3) = measureNoiseSTD_q * measureNoiseSTD_q * eye_3;

  // ========================
  // === Update procedure ===
  // ========================
  // ##Step1: Calculate the residual
  const Vector3d residual_p = measUpdate_p - StateAtUpdate->p;
  Quaterniond residual_q = measUpdate_q * StateAtUpdate_q.conjugate();
  residual_q.normalize();
  const Vector3d residual_th = 2*residual_q.vec(); // q ~= [1, 0.5*theta]
  VectorXd residualVec(6);
  residualVec << residual_p, residual_th;

  // ##Step2: Compute the Innovation, Kalman gain and Correction state
  const Matrix<double, 6, 6> S = this->eskfUtil.H * StateAtUpdate->P * this->eskfUtil.H.transpose() + R;

  // --- Set tic time --- (for inverse testing)
  auto t0_inv = std::chrono::high_resolution_clock::now();
  //const Matrix<double, 6, 6> S_inv = S.colPivHouseholderQr().inverse();
  const Matrix<double, 6, 6> S_inv = S.inverse();
  // --- Count the computation time --- (for inverse testing)
  auto t1_inv = std::chrono::high_resolution_clock::now(); // Set toc time

  const Matrix<double, 15, 6> K = StateAtUpdate->P * this->eskfUtil.H.transpose() * S_inv;
  const VectorXd x_update = K*residualVec;

  // ##Step3: Update the state
  // error state: &p, &v, &theta, &ba, &bw
  StateAtUpdate->p  = StateAtUpdate->p  + x_update.segment<3>(0); //Block containing n elements, starting at position i
  StateAtUpdate->v  = StateAtUpdate->v  + x_update.segment<3>(3);
  StateAtUpdate->ba = StateAtUpdate->ba + x_update.segment<3>(9);
  StateAtUpdate->bw = StateAtUpdate->bw + x_update.segment<3>(12);
  const Vector4d q_update(1, 0.5*x_update[6], 0.5*x_update[7], 0.5*x_update[8]); // q = [1, 0.5*theta]
  StateAtUpdate->q = this->eskfUtil.QuatMat_L(StateAtUpdate->q) * q_update;
  this->eskfUtil.QuatNormal(StateAtUpdate->q);
  //covariance: P k+1|k+1 = (I d −KH)P k+1|k (I d −KH) T +KRK T
  const Matrix<double, 15, 15> KH = eye_15 - K*this->eskfUtil.H;
  StateAtUpdate->P = KH * StateAtUpdate->P * KH.transpose()  +  K*R*K.transpose();

  // Make sure P stays symmetric.
  StateAtUpdate->P = 0.5 * (StateAtUpdate->P + StateAtUpdate->P.transpose());

  // ##Step4: Repropagate State & Covariance
  ESKF_Container<ESKF_State>::iterator_T it_end  = this->StateBuffer.GetIteratorEnd();
  ESKF_Container<ESKF_State>::iterator_T it_curr = this->StateBuffer.GetIteratorAtValue(measTime_current);
  ESKF_Container<ESKF_State>::iterator_T it_next = it_curr;
  ++it_next;

  const double INVALID_TIME = -1;
  for (; it_curr != it_end && it_next != it_end &&
         it_curr->second->timestamp != INVALID_TIME &&
         it_next->second->timestamp != INVALID_TIME;
         ++it_curr, ++it_next)
  {
    if (it_curr->second == it_next->second)
    {
      cout << "[Repropagation]: it_curr points to same state as it_next. This must not happen." << endl;
      continue;
    }
    // Break loop if EKF reset in the meantime.
    if (!this->eskfUtil.INIT_id)
      return;
    ESKF_Prop(it_next->second, it_curr->second, this->eskfUtil);
  }

  // --- Count the computation time ---
  auto t1 = std::chrono::high_resolution_clock::now(); // Set toc time

  // Broadcasting TF
  StateTFPublish(it_next->second, this->eskfUtil);

  if(this->eskfUtil.DEBUG_Mode)
  {
    boost::shared_ptr<ESKF_State> StateTest   = this->StateBuffer.GetLast();
    cout << "\n=== Measurement Update Result ===" << endl;
    cout.precision(17);
    cout << "time: " << StateTest->timestamp << endl;
    cout << "Processing Hz: " <<  1.0/(1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count()) << endl;
    cout << "p: " << StateTest->p.transpose() << endl;
    cout << "v: " << StateTest->v.transpose() << endl;
    cout << "q: " << StateTest->q.transpose() << endl;
    cout << "ba: " << StateTest->ba.transpose() << endl;
    cout << "bw: " << StateTest->bw.transpose() << endl;
    cout.precision(5);
    cout << "P:\n" << StateTest->P << endl;
    cout << "---Update related Matrix---" << endl;
    cout << "S:\n" << S << endl;
    cout << "S_inv:\n" << S_inv << endl;
    cout << "Inverse Hz: " <<  1.0/(1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1_inv-t0_inv).count()) << endl;
    cout << "K:\n" << K << endl;
    cout << "x_update: " << x_update.transpose() << endl;
    cout << "residualVec: " << residualVec.transpose() << endl << endl;

  }

  return;
}

#endif // ESKF_CORE
