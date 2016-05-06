using UnityEngine;
//using UnityEditor;
using System.Collections;

public class RaycastBullet : MonoBehaviour {
	public float damage = 0;
	public GameObject sparks;
	LineRenderer r;
	Color c;
	float width;

	// Use this for initialization
	void Shoot (Vector3 direction) {
		r  = this.GetComponent<LineRenderer>();
		Debug.DrawRay (transform.position, 100*direction);
		//EditorApplication.isPaused = true;
		bool hitted = false;
		RaycastHit[] hits;
		float minDistance = 10000;
		Transform myHit = null;
		Vector3 myHitPos = Vector3.zero;
		if ((hits = Physics.RaycastAll (transform.position, direction)).Length > 0) {
			foreach (RaycastHit hit in hits) {
				//print (hit.transform.name);
				float dist;
				if (hit.transform.tag != "Me" && (dist=Vector3.Distance(transform.position, hit.point)) < minDistance) {
					minDistance = dist;
					myHit = hit.transform;
					myHitPos = hit.point;
					hitted = true;
				}
			}

		} 
		if (!hitted) {
			//print ("no hit");
			r.SetPosition (0, transform.position);
			r.SetPosition (1, (transform.position + 10000 * direction));
		} else {
			//print (myHit.transform.name);
			r.SetPosition (0, transform.position);
			r.SetPosition (1, myHitPos);
			if (myHit.GetComponent<Hittable>())
				myHit.SendMessage ("Hit", damage);
			GameObject.Instantiate (sparks, myHitPos, Quaternion.identity);
		}
		c = new Color (1, 1, 1);
		width = .1f;
	}
	
	// Update is called once per frame
	void Update () {
		c.a -= Time.deltaTime;
		r.SetColors (c,c);
		width -= .05f * Time.deltaTime;
		r.SetWidth (width, width);
		if (width <= 0) {
			Destroy (this.gameObject);
		}
	}
}
