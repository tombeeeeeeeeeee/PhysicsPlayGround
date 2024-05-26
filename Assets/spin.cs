using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class spin : MonoBehaviour
{
    public float x = -10;
    public float y = -10;
    public float z = -10;

    Collidable col;
    // Start is called before the first frame update
    void Start()
    {
        col = gameObject.GetComponent<Collidable>();
    }

    // Update is called once per frame
    void Update()
    {
        col.angularMomentum = new float3(x, y, z);
    }
}
