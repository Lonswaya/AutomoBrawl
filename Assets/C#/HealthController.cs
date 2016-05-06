using UnityEngine;
using System.Collections;

public class HealthController : MonoBehaviour {
	public ParticleSystem smoke;
	public float health = 100;
	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
		
		if (health < 0) {
			//smoke.
			this.GetComponent<Destructable> ().SendMessage ("Explode");
			this.BroadcastMessage ("Death");
			if (!smoke.isPlaying) {
				smoke.Play ();
			}
		} else if (health < 50) {
			if (!smoke.isPlaying) {
				smoke.Play ();
			}
			float factor = (health / 50) + 1;
			smoke.startColor = new Color (1/factor, 1/factor, 1/factor);
			ParticleSystem.ColorOverLifetimeModule cl = smoke.colorOverLifetime;
			cl.color = new ParticleSystem.MinMaxGradient (new Color(1 / factor, 1 / factor, 1 / factor));
			ParticleSystem.EmissionModule em = smoke.emission;
			em.rate = new ParticleSystem.MinMaxCurve(Mathf.Pow(2 / factor, 3));

		} 
	}
	void TakeDamage(float f) {
		health -= f;
	}
	void Hit(float f) {
		health -= f;
	}
}
