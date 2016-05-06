using UnityEngine;
using System.Collections;

public class Gibbable : MonoBehaviour {
	public float mass = 500;
	// Use this for initialization
	void Start () {
	
	}
	
	public void Gib(object[] o) {
		/*foreach (Gibbable g in transform.GetComponentsInChildren<Gibbable>()) {
			//recursively tell each of the items inside you to gib
			g.Gib();
		}*/
		Collider c;
		if (c = this.GetComponent<Collider>()) {
			c.isTrigger = false;
			Rigidbody r = gameObject.AddComponent<Rigidbody> ();
			r.velocity = (Vector3)o [1];
			r.mass = mass;
			r.AddExplosionForce (300000, (Vector3)o[0], 50); 
			transform.parent = null;
			//wheeee now we floop around
		}
	}
}
