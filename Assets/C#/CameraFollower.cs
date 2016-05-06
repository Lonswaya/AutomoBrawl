using UnityEngine;
using System.Collections;

public class CameraFollower : MonoBehaviour {
	//public GameObject toFollow;
	public Transform target;
	public Transform parent;
	public Transform reticle;

	private float timeSinceSwitch;
	private int cameraMode;
	private Rigidbody myRigid;
	//private float moveSpeed;
	// Use this for initialization
	void Start () {
		myRigid = this.GetComponent<Rigidbody> ();
	}
	
	// Update is called once per frame
	void FixedUpdate () {
		myRigid.velocity = (10 * (parent.position - transform.position) );



		timeSinceSwitch += Time.deltaTime;
		if (Input.GetAxis("Target") > 0 && timeSinceSwitch > .4f) {
			cameraMode = (cameraMode + 1) % 2;
			timeSinceSwitch = 0;

		}
		Vector3 lookAngle = transform.eulerAngles;
		switch (cameraMode) {
		case 0:
			//transform.localEulerAngles = normalAngle;
			Vector3 reticleDir = reticle.position - transform.position;
			lookAngle = Vector3.RotateTowards (transform.forward, reticleDir, Time.deltaTime * timeSinceSwitch, 0);
			if (lookAngle.magnitude < 1) {
				//print (lookAngle.magnitude);
			}
			lookAngle = new Vector3 (lookAngle.x, 0, lookAngle.z);
			break;
		case 1:
			Vector3 targetDir = target.position - transform.position;
			lookAngle = Vector3.RotateTowards (transform.forward, targetDir, Time.deltaTime * timeSinceSwitch, 0);

			if ( timeSinceSwitch > 3 ) {
				lookAngle = target.position - transform.position;
			}


			break;
		
		}


		transform.rotation = Quaternion.LookRotation (lookAngle);
	}
}
