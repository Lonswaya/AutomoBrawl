using UnityEngine;
using System.Collections;
[System.Serializable]
public class TurretInfo {
	public Transform horizontalTurret;
	public Transform verticalTurret;
}
public class WeaponController : MonoBehaviour {
	public TurretInfo[] weapons;
	public bool aiming = true;


	// Use this for initialization
	void Start () {
		//Cursor.visible = false;
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

	}

}
