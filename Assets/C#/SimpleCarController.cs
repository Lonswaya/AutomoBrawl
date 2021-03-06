﻿using UnityEngine;
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
	public float timeToBoost;
	public ParticleSystem booster;
	public int InputMethod = 0; //-1 for immobile, 0 for player, 1 for AI

	private float timeSinceBoost;
	private Rigidbody myRigid;
	private WeaponController myWep;
	private float timeGrounded, timeUpsideDown;
	private bool lastDrifting;
	private bool dead;



	private float vertAxis = 0;
	private float horizAxis = 0;
	private float brakeForce = 0;
	private float driftAxis = 0;
	private bool boosting = false;
	private bool[] firing = new bool[0];

	public void Start() {
		myWep = this.GetComponent<WeaponController> ();
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
	/************************************************************************************/







	public void FixedUpdate()
	{
		ProcessInputs ();

		if (!dead) {
			FixInverted ();
			ProcessWheels ();
			ActivateWeapons ();
		}


	}

	public void ProcessInputs() {
		//are we AI or something else
		switch (InputMethod) {
		case 0:
			vertAxis = Input.GetAxis ("Vertical");
			horizAxis = Input.GetAxis ("Horizontal");
			brakeForce = Input.GetAxis ("Brakes");
			driftAxis = Input.GetAxis ("Drift");
			boosting = (Input.GetAxis ("Boost") > 0);
			firing = new bool[myWep.weapons.Length];
			int length = myWep.weapons.Length;
			for (int index = 0; index < length; index++) {
				firing[index] = (Input.GetAxis ("Fire" + (index + 1)) > 0);
			}

			break;
		case 1:
			vertAxis = 0;
			horizAxis = 0;
			brakeForce = 0;
			driftAxis = 0;
			boosting = false;
			firing = new bool[0];
			break;
		}
	}
	public void ActivateWeapons() {
		myWep.ApplyInput (firing);
	}

	public void AirMovement(float forwardRotation, float sideRotation) {
		//print ("airborne");
		myRigid.AddRelativeTorque (new Vector3 (forwardRotation * Time.deltaTime * 1000000, sideRotation * Time.deltaTime * 1000000, 0));
		//print (myRigid.rotation);
	}
	public void FixInverted() {
		float rotZ = transform.eulerAngles.z%360;
		if (rotZ >= 80 && rotZ <= 280) {
			timeUpsideDown += Time.deltaTime;
			if (timeUpsideDown > timeToFixInverted) {
				//print("lurch");
				myRigid.AddForce(400 * Vector3.up * myRigid.mass);
				myRigid.AddTorque(0,60 * myRigid.mass,0);
				timeUpsideDown = 0;
			}
		} else {
			timeUpsideDown = 0;
		}
	}
	public void ProcessWheels() {
		






		float motor = maxMotorTorque * vertAxis;
		float steering = maxSteeringAngle * horizAxis;
		bool drifting = driftAxis > 0;
		bool airborne = false;

		myRigid.AddRelativeForce(Vector3.forward * motor * Time.deltaTime * 150);
		timeSinceBoost += Time.deltaTime;
		if (boosting && timeSinceBoost > timeToBoost) { 
			myRigid.AddRelativeForce (Vector3.up * 500 * myRigid.mass);
			myRigid.AddRelativeForce(Vector3.forward * 2000 * myRigid.mass);
			timeSinceBoost = 0;
			booster.Play ();
		}

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



			ApplyLocalPositionToVisuals(axleInfo.leftWheel);
			ApplyLocalPositionToVisuals(axleInfo.rightWheel);


		}
		if (airborne) {
			//myRigid.AddRelativeTorque(new Vector3(0, 0, steering * Time.deltaTime * 60));

			AirMovement (vertAxis, horizAxis);
		}
	}
	void Death() {
		dead = true;
	}
}