using UnityEngine;
using System;


[System.Serializable]
public class WeaponValues {
	public Transform body;
	public Transform shootDir;
	public ParticleSystem particles;

	//public Vector3 FiringVector;
}


public class SimpleWeapon : MonoBehaviour, Weapon {


	//public parameters
	public int ammo = Int32.MaxValue; //Infinite for now
	public int itemsPerHit = 1;       //ex. 5 for a shotgun
	public int health = 100;          //Health if we plan on having weapons being destroyed
	public int salvoMode = 1;         //How many weapons would be shooting at once (for two weapons, 2 would mean both at once. 1 would mean each in between the other)
	public float firingSpeed = 1;     //How fast we want our items to fly
	public float coolDown = .3f;      //Weapon cooldown in between shots
	public bool automatic;			  //If it is automatically firing or not
	public int damage = 10;


	public GameObject itemToShoot;
	public WeaponValues[] weapons;
	//private values
	private int salvoIndex = 0;
	private bool firing = false;
	private float firingTime;
	private WeaponController parent;
	private Rigidbody myRigid;
	//member objects

	void Start() {
		//initialize our initial weapons
		/*if (shootPoints != null && shootPoints.Length > 0) {
			
		} else {
			Debug.LogError ("SimpleWeapon: Missing shootpoints, please delegate places for your weapon to shoot (can even be the gameobject itself)");
		}*/
		foreach (WeaponValues wv in weapons) {
			if (wv.particles == null) {
				ParticleSystem ps;
				if (ps = wv.body.GetComponent<ParticleSystem> ())
					wv.particles = ps;
			}
		}
	}
	void Update() {
		firingTime += Time.deltaTime;
	}


	void Weapon.FireDown() {
		//start firing, button down
		if (firing && !automatic) {
			//print("firing must be false");
			return;
		}
		firing = true;
		//if enough time has not passed
		if (firingTime < coolDown * salvoMode) {
			//print ("not enough time has passed");
			return;
		}
		if (ammo <= 0) {
			//print ("out of ammo");
			return;
		}
		ammo--;
		int startSalvo = salvoIndex;
		for(int index = startSalvo; index < startSalvo + salvoMode; index++) {
			WeaponValues wv = weapons [index%weapons.Length];
			for (int i = 0; i < itemsPerHit; i++) {
				//spawn gameobject where it is
				GameObject g = (GameObject)GameObject.Instantiate (itemToShoot, wv.shootDir.position, Quaternion.identity);
				g.transform.LookAt(wv.shootDir.position);
				//need to call shoot because we will look at first
				RaycastBullet rb;
				if (rb = g.GetComponent<RaycastBullet> ()) {
					rb.damage = damage;
					rb.SendMessage ("Shoot", (wv.shootDir.position - wv.body.position));
				}
				Explosive ex;
				if (ex = g.GetComponent<Explosive> ()) {
					ex.damage = damage;

				}

				Rigidbody r;
				if (r = g.GetComponent<Rigidbody> ()) {
					r.velocity = myRigid.velocity;
					r.AddForce ((wv.shootDir.position - wv.body.position) * firingSpeed * 6000);
				}
				//play particle system (if exists)
				if (wv.particles) {
					wv.particles.Play ();
				}
			}

		}
		firingTime = 0;
		salvoIndex = (salvoIndex + salvoMode)%weapons.Length;
	}
	void Weapon.FireUp() {
		//cease fire, button up
		//print("stop pew");
		if (firing) {
			firing = false;
			return;
		}
	}
	void Weapon.SetRigid(Rigidbody r) {
		myRigid = r;
	}

}
