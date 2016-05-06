using UnityEngine;
//using UnityEditor;
using System.Collections;

public class Explosive : MonoBehaviour {
	public GameObject explosion;

	public int samples = 20;
	public float distance = 5;
	public float damage = 1;

	private float time;

	void OnCollisionEnter(Collision c) {
		Explode (c.transform);
	}
	void OnTriggerEnter(Collider c) {
		Explode (c.transform);
	}
	void Explode(Transform t) {
		if (t.tag != "Me" && time > .1f) {
			//EditorApplication.isPaused = true;
			GameObject o = (GameObject)GameObject.Instantiate (explosion, transform.position, transform.rotation);
			o.GetComponent<Explosion> ().samples = samples;
			o.GetComponent<Explosion> ().distance = distance;
			o.GetComponent<Explosion> ().damage = damage;
			o.SendMessage ("Explode");
			GameObject.Destroy (this.gameObject);
		}
	}

	void Update() {
		time += Time.deltaTime;
	}
}
