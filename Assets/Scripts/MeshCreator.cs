using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MeshCreator : MonoBehaviour
{
    private List<Vector3> points;
    public GameObject poly;

    public static MeshCreator Instance()
    {
        return _singleton;
    }

    private static MeshCreator _singleton;

    // Start is called before the first frame update
    void Start()
    {
        _singleton = this;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetPoint(List<Vector3> p)
    {
        points = p;
    }

    public void GenerateMesh()
    {
        if (points.Count < 3 )
        {
            return;
        }
        
        List<Vector3> vertices = new List<Vector3>();
        List<int> indices = new List<int>();
        List<Vector3> normals = new List<Vector3>();
        List<Vector2> uv = new List<Vector2>();


        
        
        
        List<Vector3> pointSorted = points.OrderBy(x => x.x).ToList();
        List<Vector3> envelop = new List<Vector3>();
        List<Vector3> polygon = new List<Vector3>();

        //var tempList = pointSorted.GetRange(0, 3).OrderBy(x => x.transform.position.y).ToList();
        
        Arete a1 = new Arete(pointSorted[0], pointSorted[1]);
        
        polygon.Add(pointSorted[0]);
        polygon.Add(pointSorted[1]);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        //Arete a2 = CreateLine(tempList[1].transform.position, tempList[2].transform.position);

        if (points.Count < 2)
        {
            return;
        }

        for (int i = 0; i < pointSorted.Count; i++)
        {
            Vector3 v = polygon[0] - polygon[1];
            Vector3 u = polygon[1] - pointSorted[0];
            if (!ProjectManager.Instance().isColinear(v,u))
            {
                break;
            }

            new Arete(pointSorted[i], pointSorted[(i + 1) % pointSorted.Count]);
            polygon.Add(pointSorted[0]);
            pointSorted.RemoveAt(0);
        }

        while (pointSorted.Count > 0)
        {
            envelop = getConvexEnvelopJarvis(polygon);
            for (int i = 0; i < envelop.Count; i++)
            {
                var left = isLeft(envelop[i], envelop[(i+1) % envelop.Count],
                    pointSorted[0]);
                if (left)
                {
                    Arete areteA = new Arete(envelop[i], pointSorted[0]);
                    Arete areteB = new Arete(envelop[(i+1)%envelop.Count], pointSorted[0]);
                    Arete areteC = new Arete(envelop[i], envelop[(i + 1) % envelop.Count]);
                    Triangle t = new Triangle(areteA, areteB, areteC);
                    vertices.AddRange(getConvexEnvelopJarvis(t.GetSommet()).GetRange(0,3));
                }
            }
            polygon.Add(pointSorted[0]);
            pointSorted.RemoveAt(0);
        }
        

        var mesh = new Mesh {name = "MeshGenerate"};
        
        indices = Enumerable.Range(0, vertices.Count).ToList();
        normals = Enumerable.Repeat(Vector3.back, vertices.Count).ToList();

        Debug.Log($"'{vertices.Count} - {indices.Count} - {normals.Count}");
        
        
        
        mesh.vertices = vertices.ToArray();
        mesh.triangles = indices.ToArray();
        mesh.normals = normals.ToArray();
        
        

        GameObject go = (GameObject) Instantiate(poly, new Vector3(0,0,-0.5f), Quaternion.Euler(0, 0, 0));
        go.GetComponent<MeshFilter>().mesh = mesh;
        return;
    }

    List<Vector3> MeshTriangleVertices(Triangle t)
    {
        List<Vector3> triangle = t.GetSommet();
        triangle = triangle.OrderBy(x => x.x).ThenBy(x => x.y).ToList();
        var left = triangle[0];

        var list = triangle.GetRange(1, 2);
        list = list.OrderByDescending(x => x.y).ThenBy(x => x.x).ToList();
            
        var middle = list[0];
        var right = list[1];

        return new List<Vector3>() {left, middle, right};
    }

    //a = Point de la ligne
    //b = Point de la ligne
    //c = Point à tester
    public bool isLeft(Vector3 a, Vector3 b, Vector3 c){
        return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;
    }
    
    
    public List<Vector3> getConvexEnvelopJarvis(List<Vector3> points)
    {
        List<Vector3> result = new List<Vector3>();
        
        var pointMin = points[0];
        var xMin = pointMin.x;

        
        //Recherche du point le plus à gauche
        for (int i = 1; i < points.Count; i++)
        {
            var xCurrent = points[i].x;
            if (xCurrent < xMin)
            {
                pointMin = points[i];
                xMin = xCurrent;
            }
            if (Math.Abs(xCurrent - xMin) < 0.00001f)
            {
                if (points[i].y < pointMin.y)
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
            Vector3 lastPoint = new Vector3(currentPoint.x, currentPoint.y - 1, 0);
            if (result.Count > 1)
            {
                lastPoint = result[result.Count-2];
            }

            var minAngle = GetMinAngle(currentPoint,lastPoint , points);
            result.Add(minAngle);
            currentPoint = minAngle;
        } while (currentPoint != result[0]);
        
        return result;
    }

    Vector3 GetMinAngle(Vector3 currentPoint, Vector3 lastPoint, List<Vector3> points)
    {
        var minAngle = 361f;
        Vector3 minPoint = Vector3.zero;
        for (int i = 0; i < points.Count; i++)
        {
            if (points[i] != currentPoint && points[i] != lastPoint)
            {
                var angleI = GetAngle(points[i], currentPoint, lastPoint);
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
}
