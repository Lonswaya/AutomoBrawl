using UnityEngine;
using System.Collections;
[System.Serializable]
public class TurretInfo {
	
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
		

	}
	public Rigidbody GetRigid() {
		return myRigid;
	}
	public void ApplyInput(bool[] ar) {
		
		for (int i = 0; i < ar.Length; i++) {
			if (ar[i]) {
				//print (ar.IndexOf (o) + "yes");
				weaponScripts [i].FireDown ();
			} else {
				//print (ar.IndexOf (o) + "no");
				weaponScripts [i].FireUp ();
			}
		}
	}


}
