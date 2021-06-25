using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Labust.Controllers
{
	/// <summary>
	/// Vessel controller that directly controls velocity and orientation to move and rotate towards the Target position
	/// </summary>
	public class VesselScript : MonoBehaviour
	{
		public Transform Target;
		private Boolean stop;

		public void Awake()
		{
			stop = false;
		}

		public void FixedUpdate()
		{  
			rotateTowards();
			moveTowards();
		}

		public void moveTowards()
		{
			float speed = 0.5f;
			float dist = Vector3.Distance(new Vector3(Target.position.x, 0, Target.position.z), new Vector3(transform.position.x, 0, transform.position.z));
			
			//Stop the vessel when close enough to target
			if (dist < 1)
				stop = true;
			
			//start the vessel when target changed
			if (stop && dist > 5)
				stop = false;

			if (!stop)
				//use distance to slow down when approaching target position
				transform.position += transform.forward * Time.deltaTime * Mathf.Sqrt(dist) * speed;
		}

		public void rotateTowards() 
		{
			float rotationSpeed = 1.0f;
			Vector3 relativePos = Target.position - transform.position;
			Quaternion targetRotation = Quaternion.LookRotation(relativePos, Vector3.up);
			Vector3 targetAngles = targetRotation.eulerAngles;
			Quaternion target = Quaternion.Euler(0, targetAngles.y, 0);
			float error = Vector3.Angle(relativePos, transform.forward);

			if (!stop)
				//use error in interpolation ratio to minimize rotation when approaching optimal course
				transform.rotation = Quaternion.SlerpUnclamped(transform.rotation, target, Time.deltaTime * rotationSpeed);
		}
	}
}
