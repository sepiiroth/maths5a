using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Point : MonoBehaviour
{
    public Vector3 position;
    
    [SerializeField]
    private GameObject prefab;

    public Point(float x, float y, float z)
    {
        this.position = new Vector3(x, y, z);
    }

    public Point() {
        this.position = new Vector3(0, 0, 0);
    }

    public float GetX()
    {
        return position.x;
    }
    
    public float GetY()
    {
        return position.y;
    }
    
    public float GetZ()
    {
        return position.z;
    }
    
}
