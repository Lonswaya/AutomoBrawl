using UnityEngine;
//using UnityEditor;
using System.Collections;

public class Explosion : MonoBehaviour {

	public int samples = 20;
	public float distance = 5;
	public float damage = 1;
	void Explode () {
		for (int i = 0; i < 20; i++) {
			bool hitted = false;
			RaycastHit[] hits;
			float minDistance = 10000;
			Transform myHit = null;
			Vector3 myHitPos = Vector3.zero;
			Vector3 direction = Random.insideUnitSphere * distance;
			Debug.DrawRay (transform.position, direction);
			//EditorApplication.isPaused = true;
			if ((hits = Physics.RaycastAll (new Ray(transform.position, direction), distance)).Length > 0) {
				foreach (RaycastHit hit in hits) {
					//print (hit.transform.name);
					float dist;
					if ((dist=Vector3.Distance(transform.position, hit.point)) < minDistance) {
						minDistance = dist;
						myHit = hit.transform;
						myHitPos = hit.point;
						hitted = true;
					}
				}

			} 
			if (hitted) {
				print ("Hit " + myHit.name);
				if (myHit.GetComponent<Hittable>())
					myHit.SendMessage ("Hit", damage / Vector3.Distance(transform.position, myHitPos));
				Rigidbody hitsRigid;
				if (hitsRigid = myHit.GetComponent<Rigidbody> ()) {
					hitsRigid.AddExplosionForce (1000, transform.position, distance);
				}

			}
		}
	}

}
