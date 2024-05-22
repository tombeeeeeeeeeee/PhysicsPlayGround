using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OutStream : MonoBehaviour
{
    Collidable col;
    // Start is called before the first frame update
    void Start()
    {
        col = gameObject.GetComponent<Collidable>();
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(col.angularMomentum);
        //col.angularMomentum = Vector3.zero;
    }
}
