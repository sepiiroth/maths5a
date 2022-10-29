using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProjectManager : MonoBehaviour
{
    private static ProjectManager _singleton;

    public static ProjectManager Instance()
    {
        return _singleton;
    }

    private List<GameObject> pointsList;

    [SerializeField]
    private GameObject linePrefab;

    // Start is called before the first frame update
    void Start()
    {
        _singleton = this;
        pointsList = new List<GameObject>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.A))
        {
            List<GameObject> temp = getConvexEnvelopJarvis();

            var newLine = Instantiate(linePrefab, Vector3.zero, Quaternion.Euler(0, 0, 0));
            LineRenderer lR = newLine.GetComponent<LineRenderer>();
            lR.positionCount = temp.Count;
            int index = 0;
            foreach (var x in temp)
            {
               
                lR.SetPosition(index, x.transform.position + new Vector3(0,0,-0.1f)) ;
                Debug.Log(x.transform.position);
                index++;
            }

            
            //Debug.Log(temp[0].transform.position);

            //Debug.Log(GetAngle(pointsList[0].transform.position,pointsList[1].transform.position,pointsList[2].transform.position ));
        }
    }

    public void AddPointToList(GameObject point)
    {
        pointsList.Add(point);
    }

    public List<GameObject> getConvexEnvelopJarvis()
    {
        List<GameObject> result = new List<GameObject>();

        var pointMin = pointsList[0];
        var xMin = pointMin.transform.position.x;

        
        //Recherche du point le plus à gauche
        for (int i = 1; i < pointsList.Count; i++)
        {
            var xCurrent = pointsList[i].transform.position.x;
            if (xCurrent < xMin)
            {
                pointMin = pointsList[i];
                xMin = xCurrent;
            }
            if (Math.Abs(xCurrent - xMin) < 0.00001f)
            {
                if (pointsList[i].transform.position.y < pointMin.transform.position.y)
                {
                    pointMin = pointsList[i];
                    xMin = xCurrent;
                }
            }
        }
        
        
        
        result.Add(pointMin);
        //
        var currentPoint = pointMin;
        do
        {
            Vector3 lastPoint = new Vector3(currentPoint.transform.position.x, currentPoint.transform.position.y - 1, 0);
            if (result.Count > 1)
            {
                lastPoint = result[result.Count-2].transform.position;
            }

            var minAngle = GetMinAngle(currentPoint.transform.position,lastPoint );
            result.Add(minAngle);
            currentPoint = minAngle;
        } while (currentPoint != result[0]);
        
        
        return result;
    }

    GameObject GetMinAngle(Vector3 currentPoint, Vector3 lastPoint)
    {
        var minAngle = 361f;
        GameObject minPoint = null;
        Debug.Log($"{currentPoint}");
        for (int i = 0; i < pointsList.Count; i++)
        {
            if (pointsList[i].transform.position != currentPoint && pointsList[i].transform.position != lastPoint)
            {
                var angleI = GetAngle(pointsList[i].transform.position, currentPoint, lastPoint);
                Debug.Log($"{pointsList[i].transform.position}, {currentPoint}, {lastPoint} = {angleI}");
                if (angleI < minAngle)
                {
                    minAngle = angleI;
                    minPoint = pointsList[i];
                }
            }
        }

        return minPoint;
    }
    
    float GetAngle(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 u = new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        Vector3 v = new Vector3(c.x - b.x, c.y - b.y, c.z - b.z);


        float uNorm = Mathf.Sqrt(Mathf.Pow(u.x, 2) + Mathf.Pow(u.y, 2));
        float vNorm = Mathf.Sqrt(Mathf.Pow(v.x, 2) + Mathf.Pow(v.y, 2));

        
        if(uNorm == 0 | vNorm == 0)
        {
            return 0;
        }

        float dotUV = (u.x * v.x) + (u.y * v.y);
        
        var angle = Mathf.Acos(dotUV/(uNorm * vNorm)) * 180 / Mathf.PI;

        var perp = Vector3.Cross(v, u);
        
        //Test pour connaitre le sens de l'angle
        var temp = -v.x * u.y + v.y * u.x;
        
        if (temp < 0)
        {
            angle = 360 - angle;
        }

        return angle;
        //return Vector3.Angle(u, v);
    }
}


