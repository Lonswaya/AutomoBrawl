using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class AxleInfo {
	public WheelCollider leftWheel; 
	public WheelCollider rightWheel;
	public WheelFrictionCurve forwardNormal, forwardDrifting, sidewaysNormal, sidewaysDrifting;
	public SerialWheelFrictionCurve sforwardNormal, sforwardDrifting, ssidewaysNormal, ssidewaysDrifting;
	public bool motor;
	public bool steering;
}
[System.Serializable]
public class SerialWheelFrictionCurve {
	public float eSlip;
	public float eValue;
	public float aSlip;
	public float aValue;
}
[System.Serializable]
public class SimpleCarController : MonoBehaviour {
	public List<AxleInfo> axleInfos; 
	public float maxMotorTorque;
	public float maxSteeringAngle;
	public float timeToFixInverted = 2.5f;
	private Rigidbody myRigid;
	private float timeGrounded, timeUpsideDown;
	private bool lastDrifting;

	public void Start() {
		myRigid = transform.GetComponent<Rigidbody>();
		foreach (AxleInfo axleInfo in axleInfos) {
			//collision stuff
			axleInfo.leftWheel.ConfigureVehicleSubsteps(5,12,15);
			axleInfo.rightWheel.ConfigureVehicleSubsteps(5,12,15);

			//create new friction curves at runtime

			UpdateFrictionCurves(axleInfo);
		}
	}

	public void UpdateFrictionCurves(AxleInfo axleInfo) {
		axleInfo.forwardNormal = new WheelFrictionCurve();
		axleInfo.forwardNormal.extremumSlip = axleInfo.sforwardNormal.eSlip;
		axleInfo.forwardNormal.extremumValue = axleInfo.sforwardNormal.eValue;
		axleInfo.forwardNormal.asymptoteSlip = axleInfo.sforwardNormal.aSlip;
		axleInfo.forwardNormal.asymptoteValue = axleInfo.sforwardNormal.aValue;
		axleInfo.forwardNormal.stiffness = 1;

		axleInfo.forwardDrifting = new WheelFrictionCurve();
		axleInfo.forwardDrifting.extremumSlip = axleInfo.sforwardDrifting.eSlip;
		axleInfo.forwardDrifting.extremumValue = axleInfo.sforwardDrifting.eValue;
		axleInfo.forwardDrifting.asymptoteSlip = axleInfo.sforwardDrifting.aSlip;
		axleInfo.forwardDrifting.asymptoteValue = axleInfo.sforwardDrifting.aValue;
		axleInfo.forwardDrifting.stiffness = 1;

		axleInfo.sidewaysNormal = new WheelFrictionCurve();
		axleInfo.sidewaysNormal.extremumSlip = axleInfo.ssidewaysNormal.eSlip;
		axleInfo.sidewaysNormal.extremumValue = axleInfo.ssidewaysNormal.eValue;
		axleInfo.sidewaysNormal.asymptoteSlip = axleInfo.ssidewaysNormal.aSlip;
		axleInfo.sidewaysNormal.asymptoteValue = axleInfo.ssidewaysNormal.aValue;
		axleInfo.sidewaysNormal.stiffness = 1;

		axleInfo.sidewaysDrifting = new WheelFrictionCurve();
		axleInfo.sidewaysDrifting.extremumSlip = axleInfo.ssidewaysDrifting.eSlip;
		axleInfo.sidewaysDrifting.extremumValue = axleInfo.ssidewaysDrifting.eValue;
		axleInfo.sidewaysDrifting.asymptoteSlip = axleInfo.ssidewaysDrifting.aSlip;
		axleInfo.sidewaysDrifting.asymptoteValue = axleInfo.ssidewaysDrifting.aValue;
		axleInfo.sidewaysDrifting.stiffness = 1;

	}

	// finds the corresponding visual wheel
	// correctly applies the transform
	public void ApplyLocalPositionToVisuals(WheelCollider collider)
	{
		if (collider.transform.childCount == 0) {
			return;
		}

		Transform visualWheel = collider.transform.GetChild(0);

		Vector3 position;
		Quaternion rotation;
		collider.GetWorldPose(out position, out rotation);
		//print(visualWheel.name);
		visualWheel.transform.position = position;
		visualWheel.transform.rotation = rotation;
	}

	public void FixedUpdate()
	{
		FixInverted();
		ProcessWheels();


	}

	public void FixInverted() {
		float rotZ = transform.eulerAngles.z%360;
		if (rotZ > 140 && rotZ < 220) {
			timeUpsideDown += Time.deltaTime;
			if (timeUpsideDown > timeToFixInverted) {
				//print("lurch");
				myRigid.AddForce(300 * Vector3.up * myRigid.mass);
				myRigid.AddTorque(0,0,60 * myRigid.mass);
				timeUpsideDown = 0;
			}
		} else {
			timeUpsideDown = 0;
		}
	}
	public void ProcessWheels() {
		float motor = maxMotorTorque * Input.GetAxis("Vertical");
		float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
		float brakeForce = Input.GetAxis("Brakes");
		float driftAxis = Input.GetAxis("Drift");
		bool drifting = driftAxis > 0;
		bool airborne = false;

		myRigid.AddRelativeForce(Vector3.forward * motor * Time.deltaTime * 150);

		if (steering != 0) {
			myRigid.AddRelativeTorque(new Vector3(0,steering * Time.deltaTime * 300, 0));
		}

		foreach (AxleInfo axleInfo in axleInfos) {
			if (axleInfo.steering) {
				axleInfo.leftWheel.steerAngle = steering;
				axleInfo.rightWheel.steerAngle = steering;
			}
			if (axleInfo.motor) {

				if (brakeForce > 0) {
					axleInfo.leftWheel.brakeTorque = brakeForce;
					axleInfo.rightWheel.brakeTorque = brakeForce;
					axleInfo.leftWheel.motorTorque = 0;
					axleInfo.rightWheel.motorTorque = 0;
					//myRigid.AddForce(myRigid.velocity *  brakeForce * -1000 * Time.deltaTime);
				} else {
					axleInfo.leftWheel.brakeTorque = 0;
					axleInfo.rightWheel.brakeTorque = 0;
				//print(axleInfo.rightWheel.
					axleInfo.leftWheel.motorTorque = motor;
					axleInfo.rightWheel.motorTorque = motor;
				}

			}
			if (drifting != lastDrifting) {
				
				if (drifting) { //TODO'
					/* Apply drift information to each wheel
					* Only apply for sideways friction
					* Play around with values
					*/
					axleInfo.leftWheel.forwardFriction = axleInfo.forwardDrifting;
					axleInfo.leftWheel.sidewaysFriction = axleInfo.sidewaysDrifting;
					axleInfo.rightWheel.forwardFriction = axleInfo.forwardDrifting;
					axleInfo.rightWheel.sidewaysFriction = axleInfo.sidewaysDrifting;

				} else {
					//restore original values
					axleInfo.leftWheel.forwardFriction = axleInfo.forwardNormal;
					axleInfo.leftWheel.sidewaysFriction = axleInfo.sidewaysNormal;
					axleInfo.rightWheel.forwardFriction = axleInfo.forwardNormal;
					axleInfo.rightWheel.sidewaysFriction = axleInfo.sidewaysNormal;

				}
			}
			lastDrifting = drifting;


			//http://forum.unity3d.com/threads/how-to-make-a-physically-real-stable-car-with-wheelcolliders.50643/
			WheelHit hit;
			//float travelL, travelR;
			bool groundedL, groundedR;
			if (groundedL = axleInfo.leftWheel.GetGroundHit(out hit)) {
				//travelL = (-axleInfo.leftWheel.transform.InverseTransformPoint(hit.point).y - axleInfo.leftWheel.radius) / axleInfo.leftWheel.suspensionDistance;
			} else {
				//travelL = 1;
			}
			if (groundedR = axleInfo.rightWheel.GetGroundHit(out hit)) {
				//travelR = (-axleInfo.rightWheel.transform.InverseTransformPoint(hit.point).y - axleInfo.rightWheel.radius) / axleInfo.rightWheel.suspensionDistance;
			} else {
				//travelR = 1;
			}
			//float AntiRoll =5; //temporary
			//float antiRollForce = (travelL - travelR) * AntiRoll;
			if (groundedL && groundedR) {
				timeGrounded += Time.deltaTime;
				if (timeGrounded > 1.5f) {
					//had enough time to become centered/joints to settle, freeze
					transform.localEulerAngles = new Vector3(transform.localEulerAngles.x, transform.localEulerAngles.y, 0);
					myRigid.constraints = RigidbodyConstraints.FreezeRotationZ;
				}
			} else {
				myRigid.constraints = 0;
				timeGrounded = 0;
				airborne = true;
			}
			/*if (groundedL) {
				myRigid.AddForceAtPosition(axleInfo.leftWheel.transform.up * -antiRollForce, axleInfo.leftWheel.transform.position * 10); 
				Debug.DrawLine(axleInfo.leftWheel.transform.up * -antiRollForce + axleInfo.leftWheel.transform.position, axleInfo.leftWheel.transform.position);
			}
			if (groundedR) {
				myRigid.AddForceAtPosition(axleInfo.rightWheel.transform.up * antiRollForce, axleInfo.rightWheel.transform.position * 10);
				Debug.DrawLine(axleInfo.rightWheel.transform.up * -antiRollForce + axleInfo.rightWheel.transform.position, axleInfo.rightWheel.transform.position);

			}*/



			ApplyLocalPositionToVisuals(axleInfo.leftWheel);
			ApplyLocalPositionToVisuals(axleInfo.rightWheel);


		}
		if (airborne) {
			myRigid.AddRelativeTorque(new Vector3(0, 0, steering * Time.deltaTime * 60));
		}
	}
}