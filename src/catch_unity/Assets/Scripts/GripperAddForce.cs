using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripperAddForce : MonoBehaviour
{
    public float sideForce = 5f;
    private Rigidbody targetRb = null;

    void Start()
    {
        targetRb = GetComponent<Rigidbody>();
    }
    void OnCollisionEnter(Collision collision)
    {
        Rigidbody rb = collision.gameObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            targetRb = rb;
        }
    }

    void OnCollisionExit(Collision collision)
    {
        Rigidbody rb = collision.gameObject.GetComponent<Rigidbody>();
        if (rb != null && rb == targetRb)
        {
            targetRb = null;
        }
    }

    void Update()
    {
        if (Input.GetKey(KeyCode.Space))
        {
            targetRb.AddForce(transform.right * sideForce, ForceMode.Force);
        }
    }
}
