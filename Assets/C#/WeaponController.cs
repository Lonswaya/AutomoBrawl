using UnityEngine;
using System.Collections;
[System.Serializable]
public class TurretInfo {
	public Transform horizontalTurret;
	public Transform verticalTurret;
	public Transform firingBody;
}
public class WeaponController : MonoBehaviour {
	public TurretInfo[] weapons;
	public bool aiming = true;

	private Rigidbody myRigid;
	private Weapon[] weaponScripts;
	// Use this for initialization
	void Start () {
		//Cursor.visible = false;
		myRigid = transform.GetComponent<Rigidbody>();
		weaponScripts = new SimpleWeapon[weapons.Length];
		for (int i = 0; i < weapons.Length; i++) {
			weaponScripts [i] = weapons [i].firingBody.GetComponent<SimpleWeapon> ();
			weaponScripts [i].SetRigid (myRigid);
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (aiming) {
			float mouseX = Input.GetAxis("Mouse X") * 5;
			float mouseY = Input.GetAxis("Mouse Y") * -5;

			foreach (TurretInfo t in weapons) {
				t.horizontalTurret.localEulerAngles = new Vector3(t.horizontalTurret.localEulerAngles.x, t.horizontalTurret.localEulerAngles.y + mouseX, t.horizontalTurret.localEulerAngles.z);
				t.verticalTurret.localEulerAngles = new Vector3(t.verticalTurret.localEulerAngles.x + mouseY, t.verticalTurret.localEulerAngles.y, t.verticalTurret.localEulerAngles.z);
			}
		}

		for (int index = 0; index < weapons.Length; index++) {
			//print(Input.GetAxis("Fire" + (index+1)));
			if (Input.GetAxis ("Fire" + (index + 1)) > 0) {
				weaponScripts [index].FireDown ();
			} else {
				weaponScripts [index].FireUp ();
			}
		}

	}
	public Rigidbody GetRigid() {
		return myRigid;
	}
}
