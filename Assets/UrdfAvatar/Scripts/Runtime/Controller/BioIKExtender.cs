using UnityEngine;
using System.Collections.Generic;


namespace UrdfAvatar
{
    class BioIKExtender : MonoBehaviour
    {
        void Awake()
        {
            // get bioik
            var bioik = this.GetComponent<BioIK.BioIK>();
            if (bioik == null)
            {
                Debug.LogWarning("BioIK is null.");
                return;
            }

            // get articulation body compoinents
            var bodies = this.GetComponentsInChildren<ArticulationBody>();
            foreach (var body in bodies)
            {
                var segment = bioik.FindSegment(body.name);
                var bioik_joint = segment.AddJoint();

                // calc enabled axis from articulation body
                var urdf_joint = body.gameObject.GetComponent<Unity.Robotics.UrdfImporter.UrdfJoint>();
                var bioik_joint_quat = Quaternion.Euler(bioik_joint.GetOrientation());
                var dir = body.anchorRotation * bioik_joint_quat * Vector3.right;
                var is_x = Vector3.Dot(dir, Vector3.right);
                var is_y = Vector3.Dot(dir, Vector3.up);
                var is_z = Vector3.Dot(dir, Vector3.forward);

                // hoge
                // var weight = segment.AddObjective(BioIK.ObjectiveType.JointValue) as BioIK.JointValue;
                // weight.SetWeight(1.0f);

                if (Mathf.Abs(is_x) > 0.001f)
                {
                    bioik_joint.X.Enabled = true;
                    bioik_joint.X.UpperLimit = body.xDrive.upperLimit;
                    bioik_joint.X.LowerLimit = body.xDrive.lowerLimit;
                    // weight.SetXMotion(true);
                }
                else if (Mathf.Abs(is_y) > 0.001f)
                {
                    bioik_joint.Y.Enabled = true;
                    bioik_joint.Y.UpperLimit = -body.xDrive.lowerLimit;
                    bioik_joint.Y.LowerLimit = -body.xDrive.upperLimit;
                    // weight.SetYMotion(true);
                }
                else if (Mathf.Abs(is_z) > 0.001f)
                {
                    bioik_joint.Z.Enabled = true;
                    // bioik_joint.Z.UpperLimit = body.xDrive.upperLimit;
                    // bioik_joint.Z.LowerLimit = body.xDrive.lowerLimit;
                    bioik_joint.Z.UpperLimit = -body.xDrive.lowerLimit;
                    bioik_joint.Z.LowerLimit = -body.xDrive.upperLimit;
                    // weight.SetZMotion(true);
                }
            }
        }
    }
}