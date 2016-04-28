using UnityEngine;
using System.Collections;

public class CameraFollower : MonoBehaviour {
	//public GameObject toFollow;
	public Transform target;

	private Vector3 aimingAngle;

	// Use this for initialization
	void Start () {
		//aimingAngle = Vector3.back  * 10 + Vector3.up * 3;
		//toFollowTrans = toFollow.transform;
	}
	
	// Update is called once per frame
	void Update () {
		//transform.position = toFollowTrans.position;
		transform.LookAt(target.position);
		//transform.rotation = Quaternion.Euler(toFollowTrans.position - aimingAngle);
	}
}
