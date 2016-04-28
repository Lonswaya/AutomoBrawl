using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class AxleInfo {
	public WheelCollider leftWheel;
	public WheelCollider rightWheel;
	public bool motor;
	public bool steering;

}

public class SimpleCarController : MonoBehaviour {
	public List<AxleInfo> axleInfos; 
	public float maxMotorTorque;
	public float maxSteeringAngle;
	private Rigidbody myrigid;
	private float timeGrounded;

	public void Start() {
		myrigid = transform.GetComponent<Rigidbody>();
		foreach (AxleInfo axleInfo in axleInfos) {
			axleInfo.leftWheel.ConfigureVehicleSubsteps(5,12,15);
			axleInfo.rightWheel.ConfigureVehicleSubsteps(5,12,15);

		}
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
		


		float motor = maxMotorTorque * Input.GetAxis("Vertical");
		float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
		float brakeForce = Input.GetAxis("Brakes");
		float driftAxis = Input.GetAxis("Drift");
		bool drifting = driftAxis > 0;


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
					//myrigid.AddForce(myrigid.velocity *  brakeForce * -1000 * Time.deltaTime);
				} else {
					axleInfo.leftWheel.brakeTorque = 0;
					axleInfo.rightWheel.brakeTorque = 0;
				//print(axleInfo.rightWheel.
					axleInfo.leftWheel.motorTorque = motor;
					axleInfo.rightWheel.motorTorque = motor;
				}

			}
			if (drifting) { //TODO
				/* Apply drift information to each wheel
				 * Only apply for sideways friction
				 * Play around with values
				 */
			} else {
				//restore original values
			}



			//http://forum.unity3d.com/threads/how-to-make-a-physically-real-stable-car-with-wheelcolliders.50643/
			WheelHit hit;
			float travelL, travelR;
			bool groundedL, groundedR;
			if (groundedL = axleInfo.leftWheel.GetGroundHit(out hit)) {
				travelL = (-axleInfo.leftWheel.transform.InverseTransformPoint(hit.point).y - axleInfo.leftWheel.radius) / axleInfo.leftWheel.suspensionDistance;
			} else {
				travelL = 1;
			}
			if (groundedR = axleInfo.rightWheel.GetGroundHit(out hit)) {
				travelR = (-axleInfo.rightWheel.transform.InverseTransformPoint(hit.point).y - axleInfo.rightWheel.radius) / axleInfo.rightWheel.suspensionDistance;
			} else {
				travelR = 1;
			}
			//float AntiRoll =5; //temporary
			//float antiRollForce = (travelL - travelR) * AntiRoll;
			if (groundedL && groundedR) {
				timeGrounded += Time.deltaTime;
				if (timeGrounded > 1.5f) {
					//had enough time to become centered/joints to settle, freeze
					myrigid.constraints = RigidbodyConstraints.FreezeRotationZ;
				}
			} else {
				myrigid.constraints = 0;
				timeGrounded = 0;
			}
			/*if (groundedL) {
				myrigid.AddForceAtPosition(axleInfo.leftWheel.transform.up * -antiRollForce, axleInfo.leftWheel.transform.position * 10); 
				Debug.DrawLine(axleInfo.leftWheel.transform.up * -antiRollForce + axleInfo.leftWheel.transform.position, axleInfo.leftWheel.transform.position);
			}
			if (groundedR) {
				myrigid.AddForceAtPosition(axleInfo.rightWheel.transform.up * antiRollForce, axleInfo.rightWheel.transform.position * 10);
				Debug.DrawLine(axleInfo.rightWheel.transform.up * -antiRollForce + axleInfo.rightWheel.transform.position, axleInfo.rightWheel.transform.position);

			}*/



			ApplyLocalPositionToVisuals(axleInfo.leftWheel);
			ApplyLocalPositionToVisuals(axleInfo.rightWheel);


		}
	}
}