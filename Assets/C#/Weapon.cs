using UnityEngine;
using System.Collections;

public interface Weapon
{
	//fire button down
	void FireDown ();
	//fire button up
	void FireUp ();
	//Used in the beginning, if your item needs the rigidbody (for example, setting velocity to current velocity)
	void SetRigid (Rigidbody r);
}
