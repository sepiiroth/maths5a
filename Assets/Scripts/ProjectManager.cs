using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
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
        //Envelope Convex
        if (Input.GetKeyDown(KeyCode.A))
        {
            List<GameObject> temp = getConvexEnvelopJarvis(pointsList);

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
        }
        
        //Triangulation
        if (Input.GetKeyDown(KeyCode.Z))
        {
            IncrementalTriangulation();
        }
    }

    public void AddPointToList(GameObject point)
    {
        pointsList.Add(point);
    }
    
    /*********** Enveloppe Convex *************/

    public List<GameObject> getConvexEnvelopJarvis(List<GameObject> points)
    {
        List<GameObject> result = new List<GameObject>();
        
        var pointMin = points[0];
        var xMin = pointMin.transform.position.x;

        
        //Recherche du point le plus à gauche
        for (int i = 1; i < points.Count; i++)
        {
            var xCurrent = points[i].transform.position.x;
            if (xCurrent < xMin)
            {
                pointMin = points[i];
                xMin = xCurrent;
            }
            if (Math.Abs(xCurrent - xMin) < 0.00001f)
            {
                if (points[i].transform.position.y < pointMin.transform.position.y)
                {
                    pointMin = points[i];
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

            var minAngle = GetMinAngle(currentPoint.transform.position,lastPoint , points);
            result.Add(minAngle);
            currentPoint = minAngle;
        } while (currentPoint != result[0]);
        
        return result;
    }

    GameObject GetMinAngle(Vector3 currentPoint, Vector3 lastPoint, List<GameObject> points)
    {
        var minAngle = 361f;
        GameObject minPoint = null;
        for (int i = 0; i < points.Count; i++)
        {
            if (points[i].transform.position != currentPoint && points[i].transform.position != lastPoint)
            {
                var angleI = GetAngle(points[i].transform.position, currentPoint, lastPoint);
                if (angleI < minAngle)
                {
                    minAngle = angleI;
                    minPoint = points[i];
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
    
    /************************/
    
    /************ Trinagulation Incrémentale *************/
    void IncrementalTriangulation()
    {
        List<GameObject> pointSorted = pointsList.OrderBy(x => x.transform.position.x).ToList();
        List<GameObject> envelop = new List<GameObject>();
        List<GameObject> polygon = new List<GameObject>();

        var tempList = pointSorted.GetRange(0, 3).OrderBy(x => x.transform.position.y).ToList();

        CreateLine(tempList[0].transform.position, tempList[1].transform.position);
        CreateLine(tempList[1].transform.position, tempList[2].transform.position);

        for (int i = 0; i < 3; i++)
        {
            CreateLine(tempList[i].transform.position, pointSorted[3].transform.position);
        }
        
        polygon.Add(pointSorted[0]);
        polygon.Add(pointSorted[1]);
        polygon.Add(pointSorted[2]);
        polygon.Add(pointSorted[3]);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        
        envelop = getConvexEnvelopJarvis(polygon);

        while (pointSorted.Count > 0)
        {
            envelop = getConvexEnvelopJarvis(polygon);
            for (int i = 0; i < envelop.Count; i++)
            {
                var left = isLeft(envelop[i].transform.position, envelop[(i+1) % envelop.Count].transform.position,
                    pointSorted[0].transform.position);
                if (left)
                {
                    CreateLine(envelop[i].transform.position, pointSorted[0].transform.position);
                    CreateLine(envelop[(i+1)%envelop.Count].transform.position, pointSorted[0].transform.position);
                }
            }
            polygon.Add(pointSorted[0]);
            pointSorted.RemoveAt(0);
        }
    }

    void CreateLine(Vector3 positionA, Vector3 positionB)
    {
        var lineTemp = Instantiate(linePrefab, Vector3.zero, Quaternion.Euler(0, 0, 0));
        LineRenderer lrTemp = lineTemp.GetComponent<LineRenderer>();
        lrTemp.positionCount = 2;
        lrTemp.SetPosition(0, positionA);
        lrTemp.SetPosition(1, positionB);
    }
    
    
    //a = Point de la ligne
    //b = Point de la ligne
    //c = Point à tester
    public bool isLeft(Vector3 a, Vector3 b, Vector3 c){
        return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;
    }
    
}


