
using UnityEngine;
using System.Collections;

public class Destructable : MonoBehaviour
{
	public GameObject explosion; //one gameobject holding many particle systems
	private bool destroyed = false;
	void Explode() {
		if (!destroyed) {
			print ("boom");
			object[] o = new object[2];
			Rigidbody r = this.GetComponent<Rigidbody> ();
			o [0] = (object)(transform.position + 5*Vector3.up);
			o [1] = (object)r.velocity;
			this.BroadcastMessage ("Gib",o);
			destroyed = true;
			r.isKinematic = true;
			explosion.SetActive(true);
		}
	
	}
}
