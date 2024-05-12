using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class spin : MonoBehaviour
{
    public float x = -10;
    public float y = -10;
    public float z = -10;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.rotation *= Quaternion.Euler(x * Time.deltaTime, y * Time.deltaTime, z * Time.deltaTime);
    }
}
