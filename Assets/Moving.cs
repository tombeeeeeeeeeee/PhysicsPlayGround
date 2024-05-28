using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Moving : MonoBehaviour
{
    public Vector3 vel;

    Collidable col;

    // Start is called before the first frame update
    void Start()
    {
        col = gameObject.GetComponent<Collidable>();
        col.velocity = vel;
    }

    // Update is called once per frame
    void Update()
    {
    }
}
