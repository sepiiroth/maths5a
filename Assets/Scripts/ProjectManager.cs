using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class ProjectManager : MonoBehaviour
{
    private static ProjectManager _singleton;

    public static ProjectManager Instance()
    {
        return _singleton;
    }

    private List<GameObject> pointsList;
    private List<Arete> A;
    private List<Triangle> T;
    private List<Tetraèdre> F;
    private List<Point> centreCircleList = new List<Point>();
    private List<Arete> areteStar = new List<Arete>();

    [SerializeField]
    private GameObject linePrefab;

    [SerializeField]
    private GameObject linePrefabVoronoi;

    [SerializeField]
    private GameObject barycentrePrefab;

    [SerializeField]
    private GameObject centerCircle;
    
    [SerializeField]
    private Material lineMatColorBlue;
    
    [SerializeField]
    private Material lineMatColorPurple;

    [SerializeField]
    private Material lineMatColorRed;
    
    GameObject point0;

    // Start is called before the first frame update
    void Start()
    {
        _singleton = this;
        pointsList = new List<GameObject>();
        A = new List<Arete>();
        T = new List<Triangle>();
        F = new List<Tetraèdre>();
    }

    // Update is called once per frame
    void Update()
    {
        //Envelope Convex
        if (Input.GetKeyDown(KeyCode.A))
        {
            List<GameObject> temp = getConvexEnvelopJarvis(pointsList);

            for (int i = 0; i < temp.Count; i++)
            {
                CreateLine(temp[i].transform.position, temp[(i + 1) % temp.Count].transform.position);
            }
        }

        if(Input.GetKeyDown(KeyCode.G)) {
            List<GameObject> temp = getConvexEnvelopGrahamScan(pointsList);

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

        if(Input.GetKeyDown(KeyCode.V)) {
            Voronoi(T, A, pointsList);
        }
        
        //Triangulation
        if (Input.GetKeyDown(KeyCode.Z))
        {
            IncrementalTriangulation();
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            Flipping();
        }
        
        if (Input.GetKeyDown(KeyCode.Q))
        {
            List<Triangle> ACq = get3DConvexEnvelop(pointsList.Select(x => x.transform.position).ToList());
            foreach (var t in ACq)
            {
                t.GetAllArete().ForEach(a => a.GetLine().GetComponent<LineRenderer>().material = lineMatColorPurple);
            }
        }

        
        if (Input.GetKeyDown(KeyCode.S))
        {
            MeshCreator.Instance().SetPoint(pointsList.Select(x => x.transform.position).ToList());
            MeshCreator.Instance().GenerateMesh();
        }
    }

    public void AddPointToList(GameObject point)
    {
        //pointsList.Add(point);
        //AjoutDelauney(point);
        AjoutDelauney3D(point);
        //IncrementalTriangulation();
        //Flipping();
    }
    
    public void RemovePointToList(GameObject point)
    {
        //RemoveDelauney(point);
        //IncrementalTriangulation();
        //Flipping();
    }

    public void Voronoi(List<Triangle> triangle, List<Arete> arete, List<GameObject> point) {
        for(int i = 0; i < triangle.Count; i++) { //Determiner le centre CT du cercle cicronscrit au triangle T
            List<Vector3> sommets = triangle[i].GetSommet();
    
            Point milieu = new Point((sommets[0][0] + sommets[1][0])/2, (sommets[0][1] + sommets[1][1])/2, 0);
            float a = -((sommets[1][0] - sommets[0][0])/(sommets[1][1] - sommets[0][1]));
            var b = - (a * milieu.GetX()) + milieu.GetY();

            Point milieu2 = new Point((sommets[1][0] + sommets[2][0])/2, (sommets[1][1] + sommets[2][1])/2, 0);
            float ap = -((sommets[2][0] - sommets[1][0])/(sommets[2][1] - sommets[1][1]));
            var bp = milieu2.GetY() - (ap * milieu2.GetX());

            var x = (bp - b)/(a - ap);
            var y  = (a * x) + b;

            Point centreCircleP = new Point(x, y, 0);

            centreCircleList.Add(centreCircleP);

            var center = Instantiate(centerCircle, centreCircleP.position, Quaternion.Euler(0, 0, 0));

            triangle[i].centreCirconscrit = centreCircleP;
        }
    
        for(int i = 0; i < arete.Count; i++) {  //Determiner l'arete aStar de l'arete a
            List<int> index = new List<int>();
            int much = 0;
            
            for(int j = 0; j < triangle.Count; j++) { // Dans combien de triangles l'arete est utilise ?
                if(triangle[j].GetAllArete().Contains(arete[i])) {
                    much++;
                    index.Add(j);                   
                }
            }

            if(much > 1) { // arete interne donc aStar = 2 centreCircle
                Arete ar = new Arete(triangle[index[0]].centreCirconscrit.position, triangle[index[1]].centreCirconscrit.position);
                Arete arL = CreateLineVoronoi(ar.GetPointA(), ar.GetPointB());
                arete[i].areteStar = ar;
                areteStar.Add(ar);
            } else if(much != 0) { // arete externe donc aStar centreCircle, milieuArete
                Point milieu = new Point((arete[i].GetPointA()[0] + arete[i].GetPointB()[0])/2, (arete[i].GetPointA()[1] + arete[i].GetPointB()[1])/2, 0);
                
                if(triangle[index[0]].ContainsPoint(triangle[index[0]].centreCirconscrit.position)) { // Le point est dans le triangle
                    Arete ar = new Arete(triangle[index[0]].centreCirconscrit.position, triangle[index[0]].centreCirconscrit.position + (milieu.position - triangle[index[0]].centreCirconscrit.position) * 30); 
                    Arete arL = CreateLineVoronoi(ar.GetPointA(), ar.GetPointB());
                    arete[i].areteStar = ar;
                    areteStar.Add(ar);
                } else {
                    Vector3 vecDir = new Vector3(triangle[index[0]].centreCirconscrit.position[0] - milieu.position[0], triangle[index[0]].centreCirconscrit.position[1] - milieu.position[1], 0);
                    if(PolygonContainsPoint(triangle[index[0]].centreCirconscrit.position)) { // Le point est dans le polygone
                        Arete ar = new Arete(triangle[index[0]].centreCirconscrit.position, (milieu.position-vecDir) * 30); 
                        Arete arL = CreateLineVoronoi(ar.GetPointA(), ar.GetPointB());
                        arete[i].areteStar = ar;
                        areteStar.Add(ar);
                    } else { 
                        Arete ar = new Arete(triangle[index[0]].centreCirconscrit.position, (triangle[index[0]].centreCirconscrit.position+vecDir) * 30); 
                        Arete arL = CreateLineVoronoi(ar.GetPointA(), ar.GetPointB());
                        arete[i].areteStar = ar;
                        areteStar.Add(ar);
                    }
                }
            }
        }

        for(int i = 0; i < point.Count; i++) { // XXX
            List<Vector3> pointsRegionList = new List<Vector3>();

            for(int j = 0; j < arete.Count; j++) {
                if(arete[j].GetAllPoints().Contains(point[i].transform.position)) {
                    pointsRegionList.Add(arete[j].areteStar.GetPointA());
                    pointsRegionList.Add(arete[j].areteStar.GetPointB()); 
                }
            }

            

            MeshCreator.Instance().SetPoint(pointsRegionList.Distinct().ToList());
            MeshCreator.Instance().GenerateMesh();
            
        }
    }

    public bool PolygonContainsPoint(Vector3 P)
    {
        List<Vector3> triangleSorted = MeshCreator.Instance().getConvexEnvelopJarvis(pointsList.Select(x=>x.transform.position).ToList());

        for (int i = 0; i < triangleSorted.Count-1; i++)
        {
            if (ProjectManager.Instance().isLeft(triangleSorted[i], triangleSorted[(i+1)], P))
            {
                return false;
            }
        }

        return true;
    }
    
    /*********** Enveloppe Convex *************/

    public void GrahamScan(ref List<GameObject> pts, ref List<GameObject> selPts) {
        if(pts.Count > 0 ) { // Point dans la liste ?
            var pt = pts[0]; // current point position

            if(selPts.Count <= 1) {
                selPts.Add(pt); // On ajoute le premier pts car le point le plus proche du referentiel fera toujours partie de l'enveloppe convexe
                pts.RemoveAt(0); // On retire le point sur l'autre liste
            } else {
                var pt1 = selPts[selPts.Count - 1];//previous point position, last point
                var pt2 = selPts[selPts.Count - 2]; // second last point
                Vector3 dir1 = pt1.transform.position - pt2.transform.position; //vec2 to vec1
                Vector3 dir2 = pt.transform.position - pt1.transform.position;//last selected point to current point position
                var cross = Vector3.Cross(dir1, dir2);// cross product
                if(cross[2] < 0) { // tournant a droite 
                    selPts.RemoveAt(selPts.Count - 1);
                } else { // tournant a gauche on passe au point suivant
                    selPts.Add(pt); 
                    pts.RemoveAt(0);
                }
            }
        }
    }

    public List<GameObject> getConvexEnvelopGrahamScan(List<GameObject> points) {
        List<GameObject> result = points;

        result = result.OrderBy(pt => pt.transform.position.y).ToList(); // Liste triee du y min vers le y max
        result = result.OrderBy(pt => Math.Atan2(pt.transform.position.y - result[0].transform.position.y, pt.transform.position.x - result[0].transform.position.x)).ToList(); // Tri en fonction de l'angle de chaque point par rapport au referentiel (ici, point le plus bas) 
        
        List<GameObject> selPts = new List<GameObject>();


        while(result.Count > 0) {
            GrahamScan(ref result, ref selPts);
        };

        selPts.Add(selPts[0]);
        
        return selPts;
    }

    public List<GameObject> getConvexEnvelopJarvis(List<GameObject> points)
    {
        if (points.Count == 2)
        {
            return points;
        }
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
            Vector3 lastPoint = new Vector3(
                currentPoint.transform.position.x, 
                currentPoint.transform.position.y - 1,
                0
                );
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
        A.ForEach(x => Destroy(x.GetLine()));
        A.Clear();
        T.Clear();


        List<GameObject> pointSorted = pointsList.OrderBy(x => x.transform.position.x).ToList();
        List<GameObject> envelop = new List<GameObject>();
        List<GameObject> polygon = new List<GameObject>();

        //var tempList = pointSorted.GetRange(0, 3).OrderBy(x => x.transform.position.y).ToList();
        
        Arete a1 = CreateLine(pointSorted[0].transform.position, pointSorted[1].transform.position);
        
        polygon.Add(pointSorted[0]);
        polygon.Add(pointSorted[1]);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        //Arete a2 = CreateLine(tempList[1].transform.position, tempList[2].transform.position);

        if (pointsList.Count < 2)
        {
            return;
        }

        for (int i = 0; i < pointSorted.Count; i++)
        {
            Vector3 v = polygon[0].transform.position - polygon[1].transform.position;
            Vector3 u = polygon[1].transform.position - pointSorted[0].transform.position;
            if (!isColinear(v,u))
            {
                break;
            }

            CreateLine(pointSorted[i].transform.position, pointSorted[(i + 1) % pointSorted.Count].transform.position);
            polygon.Add(pointSorted[0]);
            pointSorted.RemoveAt(0);
        }

        while (pointSorted.Count > 0)
        {
            envelop = getConvexEnvelopJarvis(polygon);
            for (int i = 0; i < envelop.Count; i++)
            {
                var left = isLeft(envelop[i].transform.position, envelop[(i+1) % envelop.Count].transform.position,
                    pointSorted[0].transform.position);
                if (left)
                {
                    Arete areteA = CreateLine(envelop[i].transform.position, pointSorted[0].transform.position);
                    Arete areteB = CreateLine(envelop[(i+1)%envelop.Count].transform.position, pointSorted[0].transform.position);
                    Arete areteC = GetAreteFromList(envelop[i].transform.position,
                        envelop[(i + 1) % envelop.Count].transform.position);
                    Triangle t = new Triangle(areteA, areteB, areteC);
                    T.Add(t);
                }
            }
            polygon.Add(pointSorted[0]);
            pointSorted.RemoveAt(0);
        }
        
    }

    Arete GetAreteFromList(Vector3 x, Vector3 y)
    {
        Arete temp = new Arete(x, y);
        for (int i = 0; i < A.Count; i++)
        {
            if (temp.Equals(A[i]))
            {
                return A[i];
            }
        }
        
        return null;
    }

    Arete CreateLine(Vector3 positionA, Vector3 positionB)
    {
        Arete temp = new Arete(positionA, positionB);
        if (!A.Exists(x => x.Equals(temp)))
        {
            var lineTemp = Instantiate(linePrefab, Vector3.zero, Quaternion.Euler(0, 0, 0));
            LineRenderer lrTemp = lineTemp.GetComponent<LineRenderer>();
            lrTemp.positionCount = 2;
            lrTemp.SetPosition(0, positionA);
            lrTemp.SetPosition(1, positionB);
        
        
            temp.SetLineRenderer(lrTemp);
            A.Add(temp);
        }
        else
        {
            temp = A.Find(x => x.Equals(temp));
        }
        
        return temp;
    }

    Arete CreateLineVoronoi(Vector3 positionA, Vector3 positionB)
    {
        Arete temp = new Arete(positionA, positionB);
        if (!A.Exists(x => x.Equals(temp)))
        {
            var lineTemp = Instantiate(linePrefabVoronoi, Vector3.zero, Quaternion.Euler(0, 0, 0));
            LineRenderer lrTemp = lineTemp.GetComponent<LineRenderer>();
            lrTemp.positionCount = 2;
            lrTemp.SetPosition(0, positionA);
            lrTemp.SetPosition(1, positionB);
        
        
            temp.SetLineRenderer(lrTemp);
            A.Add(temp);
        }
        else
        {
            temp = A.Find(x => x.Equals(temp));
        }
        
        return temp;
    }
    
    
    //a = Point de la ligne
    //b = Point de la ligne
    //c = Point à tester
    public bool isLeft(Vector3 a, Vector3 b, Vector3 c){
        return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;
    }
    
    //Flipping
    void Flipping()
    {
        List<Arete> Ac = A.ToList();
        while (Ac.Count > 0)
        {
            var a = Ac[0];
            Ac.RemoveAt(0);
            Vector3 S1 = a.GetPointA();
            Vector3 S2 = a.GetPointB();
            
            List<Triangle> temp = GetTriangleFromArete(a);
            //Debug.Log("-----------------------------");

            if (temp.Count < 2)
            {
                continue;
            }
            
            

            Triangle T1 = temp[0];
            Triangle T2 = temp[1];
            
            List<Vector3> pointTA = T1.GetSommet();
            List<Vector3> pointTB = T2.GetSommet();

            pointTA.Remove(a.GetPointA());
            pointTA.Remove(a.GetPointB());
        
            pointTB.Remove(a.GetPointA());
            pointTB.Remove(a.GetPointB());

            Vector3 S3 = pointTA[0];
            Vector3 S4 = pointTB[0];
            

            List<Arete> areteT1 = T1.GetAllArete();
            areteT1.Remove(a);
            List<Arete> areteT2 = T2.GetAllArete();
            areteT2.Remove(a);
            
            Arete A1 = areteT1[0];
            Arete A2 = areteT1[1];
            Arete A3 = areteT2[0];
            Arete A4 = areteT2[1];

            if (!CheckDelaunayCriteria(a, S3, S4))
            {
                a.SetPointA(S3);
                a.SetPointB(S4);

                List<Arete> A1234 = new List<Arete>() {A1, A2, A3, A4};

                List<Arete> newT1 = A1234.Where(x => x.GetPointA().Equals(S1) | x.GetPointB().Equals(S1)).ToList();
                List<Arete> newT2 = A1234.Where(x => x.GetPointA().Equals(S2) | x.GetPointB().Equals(S2)).ToList();

                if (newT1.Count < 2 | newT2.Count < 2)
                {
                    Debug.Log(T1.ToString());
                    Debug.Log(T2.ToString());
                }

                T.Remove(T1);
                T.Remove(T2);

                T1 = new Triangle(a, newT1[0], newT1[1]);
                T2 = new Triangle(a, newT2[0], newT2[1]);
                
                T.Add(T1);
                T.Add(T2);
                
                Ac.Add(A1);
                Ac.Add(A2);
                Ac.Add(A3);
                Ac.Add(A4);
            }
            
        }
    }

    public bool CheckDelaunayCriteria(Arete a, Vector3 DA, Vector3 DB)
    {
        //Autre methode mais ne fonctionne pas
        /*List<Vector3> pointsT1 = new List<Vector3>(){a.GetPointA(), a.GetPointB(), DA}.OrderBy(x => x.x).ToList();
        List<Vector3> pointsT2 = new List<Vector3>(){a.GetPointA(), a.GetPointB(), DB}.OrderBy(x => x.x).ToList();

        var leftA = pointsT1[0];
        var middleA = pointsT1[1].y < pointsT1[2].y ? pointsT1[1] : pointsT1[2];
        var rightA = pointsT1[1].y < pointsT1[2].y ? pointsT1[2] : pointsT1[1];
        
        var leftB = pointsT2[0];
        var middleB = pointsT2[1].y < pointsT2[2].y ? pointsT2[1] : pointsT2[2];
        var rightB = pointsT2[1].y < pointsT2[2].y ? pointsT2[2] : pointsT2[1];
        
        //Debug.Log($"TA = {MatDet(leftA, middleA, rightA, DB)} - TB = {MatDet(leftB, middleB, rightB, DA)}");
        return MatDet(leftA, middleA, rightA, DB) > 0 & MatDet(leftB, middleB, rightB, DA) > 0;*/

        float angleT1 = GetAngle(a.GetPointA(),DA, a.GetPointB());
        float angleT2 = GetAngle(a.GetPointA(),DB, a.GetPointB());

        angleT1 = angleT1 > 180 ? 360 - angleT1 : angleT1;
        angleT2 = angleT2 > 180 ? 360 - angleT2 : angleT2;

        return angleT1 + angleT2 <= 180;
    }

    public float MatDet(Vector3 A, Vector3 B, Vector3 C, Vector3 D)
    {
        float a = A.x - D.x;
        float b = A.y - D.y;
        float c = (Mathf.Pow(A.x, 2) - Mathf.Pow(D.x, 2)) + (Mathf.Pow(A.y, 2) - Mathf.Pow(D.y, 2));
        
        float d = B.x - D.x;
        float e = B.y - D.y;
        float f = (Mathf.Pow(A.x, 2) - Mathf.Pow(D.x, 2)) + (Mathf.Pow(A.y, 2) - Mathf.Pow(D.y, 2));
        
        float g = C.x - D.x;
        float h = C.y - D.y;
        float i = (Mathf.Pow(A.x, 2) - Mathf.Pow(D.x, 2)) + (Mathf.Pow(A.y, 2) - Mathf.Pow(D.y, 2));

        return (a * e * i) - (a * f * h) + (b * f * g) - (b * d * i) + (c * d * h) - (c * e * g);
    }

    public List<Triangle> GetTriangleFromArete(Arete A)
    {
        List<Triangle> temp = new List<Triangle>();
        foreach (var triangle in T)
        {
            if (triangle.ContainsArete(A))
            {
                temp.Add(triangle);
                /*foreach (var s in triangle.GetSommet())
                {
                    Debug.Log(s);
                }*/
            }
        }

        
        return temp;
    }

    public Triangle GetTriangleContainingPoint(Vector3 P)
    {
        foreach (var currentT in T)
        {
            if (currentT.ContainsPoint(P))
            {
                return currentT;
            }
        }

        return null;
    }
    
    public bool isColinear(Vector3 v, Vector3 u)
    {
        return Vector3.Cross(v, u) == Vector3.zero;
    }

    public void AjoutDelauney(GameObject P)
    {
        if (pointsList.Count == 0)
        {
            pointsList.Add(P);
            return;
        }
        
        List<GameObject> pointSorted = pointsList.OrderBy(x => x.transform.position.x).ToList();
        List<GameObject> polygon = new List<GameObject>();

        if (pointsList.Count == 1 )
        {
            CreateLine(pointSorted[0].transform.position, P.transform.position);
        
            pointsList.Add(P);
            polygon.Add(pointSorted[0]);
            polygon.Add(P);
            pointSorted.RemoveAt(0);
            return;
        }
        Vector3 v = pointSorted[0].transform.position - pointSorted[1].transform.position;
        Vector3 u = pointSorted[1].transform.position - P.transform.position;
        if (isColinear(v,u))
        {
            GameObject nearest = pointSorted.OrderBy(x => Vector3.Distance(x.transform.position, P.transform.position))
                .First();
            CreateLine(nearest.transform.position, P.transform.position);
            pointsList.Add(P);
            return;
        }
        
        List<GameObject> envelop = new List<GameObject>();

        if (T.Count == 0)
        {
            for (int i = 0; i < pointSorted.Count-1; i++)
            {
                Arete areteA = CreateLine(pointSorted[i].transform.position, P.transform.position);
                Arete areteB = CreateLine(pointSorted[(i+1)].transform.position, P.transform.position);
                Arete areteC = CreateLine(pointSorted[i].transform.position,
                    pointSorted[(i + 1)].transform.position);
                Triangle t = new Triangle(areteA, areteB, areteC);
                T.Add(t);
            }
            pointsList.Add(P);
            return;
        }

        Triangle insider = GetTriangleContainingPoint(P.transform.position);

        List<Arete> L = new List<Arete>();
        
        if (insider != null)
        {
            List<Arete> arete = insider.GetAllArete().ToList();
            //sommet = MeshCreator.Instance().getConvexEnvelopJarvis(sommet);
            
            
            L.AddRange(arete);
            
            T.Remove(insider);
        }
        else
        {
            L = AreteSeenBy(P.transform.position);
            
        }
        
        while (L.Count > 0)
        {
            Arete a = L[0];
            if (IsInsideCircle(a,P.transform.position))
            {
                List<Triangle> list = GetTriangleFromArete(a);
                for (int i = 0; i < list.Count; i++)
                {
                    var sommetT = list[i].GetAllArete();
                    sommetT.Remove(a);
                    L.AddRange(sommetT);
                    Destroy(a.GetLine());
                    A.Remove(a);
                    T.Remove(list[i]);
                }
                
            }
            else
            {
                Arete areteA = CreateLine(a.GetPointA(), P.transform.position);
                Arete areteB = CreateLine(a.GetPointB(), P.transform.position);
                Triangle t = new Triangle(areteA, areteB, a);
                T.Add(t);
            }
            L.Remove(a);
        }
        pointsList.Add(P);

    }
    
    public void RemoveDelauney(GameObject P)
    {
        if (pointsList.Count == 1 )
        {
            pointsList.Remove(P);
            Destroy(P);
            return;
        }
        
        List<GameObject> pointSorted = pointsList.OrderBy(x => x.transform.position.x).ToList();
        List<GameObject> polygon = new List<GameObject>();

        List<Arete> areteSupp = A.Where(x => x.Contains(P.transform.position)).ToList();
        if (areteSupp.Count() == 1)
        {
            Destroy(areteSupp[0].GetLine());
            A.Remove(areteSupp[0]);
            pointsList.Remove(P);
            return;
        }
        
        if (areteSupp.Count() == 2)
        {
            var a = areteSupp[0].GetAllPoints().Where(x => x != P.transform.position).First();
            var b = areteSupp[1].GetAllPoints().Where(x => x != P.transform.position).First();
            
            Arete newArete = CreateLine(a,b);
            Destroy(areteSupp[0].GetLine());
            A.Remove(areteSupp[0]);
            Destroy(areteSupp[1].GetLine());
            A.Remove(areteSupp[1]);
            
            pointsList.Remove(P);
            return;
        }

        List<Arete> La1 = A.Where(x => x.Contains(P.transform.position)).ToList();;
        
        List<Triangle> Lt = new List<Triangle>();
        La1.ForEach(x => Lt.AddRange(GetTriangleFromArete(x)));
        Lt = Lt.Distinct().ToList();
        
        List<Arete> La2 = new List<Arete>();
        Lt.ForEach(x => La2.AddRange(x.GetAllArete()));
        La2 = La2.Distinct().ToList();
        La2 = La2.Where(x => !La1.Contains(x)).ToList();

        for (int i = 0; i < La1.Count; i++)
        {
            Destroy(La1[i].GetLine());
            A.Remove(La1[i]);
        }
        
        for (int i = 0; i < Lt.Count; i++)
        {
            T.Remove(Lt[i]);
        }

        pointsList.Remove(P);

        List<Vector3> Ls = new List<Vector3>();
        La2.ForEach(x => Ls.AddRange(x.GetAllPoints()));
        Ls = Ls.Distinct().ToList();

        
        List<Vector3> sommets;

        if (La2.Count == Ls.Count)//Fermé
        {
            while(La2.Count > 3)
            {
                sommets = new List<Vector3>();
                La2.ForEach(x=> sommets.AddRange(x.GetAllPoints()));
                sommets = sommets.OrderByDescending(x => x.y).Distinct().ToList();
                var s = sommets[0];
                Arete a1 = La2.Where(x => x.Contains(s)).First();
                Arete a2 = null;
                Triangle t = null;
                La2.Remove(a1);
                List<Arete> temp = La2.Where(x => (x.Contains(s) && !x.Equals(a1))).ToList();
                //a2 = temp[0];
                for (int i = 0; i < temp.Count; i++)
                { 
                    List<Vector3> posTemp = new List<Vector3>();
                    posTemp.AddRange(a1.GetAllPoints());
                    posTemp.AddRange(temp[i].GetAllPoints());
                    posTemp = posTemp.Distinct().ToList();

                    posTemp = MeshCreator.Instance().getConvexEnvelopJarvis(posTemp);
                    Arete k1 = new Arete(posTemp[0], posTemp[1]);
                    Arete k2 = new Arete(posTemp[1], posTemp[2]);
                    Arete k3 = new Arete(posTemp[2], posTemp[0]);
                    t = new Triangle(k1, k2, k3);

                    var center = GetCentreCercleCirconscrit(t);
                    var rayon = Mathf.Abs(Vector3.Distance(center, posTemp[0]));

                    var check = false; 

                    foreach (var x in La2.Select(x => x.GetAllPoints()).Distinct())
                    {
                        if (x.All(y => rayon < Vector3.Distance(center, y)))
                        {
                            check = true;
                            break;
                        }

                    }
                    
                    if (check)
                    {
                        a2 = temp[i];
                        break;
                    }
                }

                if (a2 == null)
                {
                    a2 = temp[0];
                }
                List<Vector3> sommetsTriangle = new List<Vector3>();
                sommetsTriangle.AddRange(a1.GetAllPoints());
                sommetsTriangle.AddRange(a2.GetAllPoints());
                sommetsTriangle = sommetsTriangle.Distinct().ToList();
                
                t = new Triangle(CreateLine(sommetsTriangle[0], sommetsTriangle[1]), CreateLine(sommetsTriangle[1], sommetsTriangle[2]), CreateLine(sommetsTriangle[2], sommetsTriangle[0]));
                var a3search = t.GetAllArete();
                a3search.RemoveAll(x => La2.Contains(x));
                a3search.Remove(a1);
                Debug.Log(a3search.Count);
                Arete a3 = new Arete(a3search[0].GetPointA(), a3search[0].GetPointB());
                
                T.Add(t);
                a3 = CreateLine(a3.GetPointA(), a3.GetPointB());
                La2.Remove(a2);
                La2.Add(a3);
                
            }
            T.Add(new Triangle(La2[0], La2[1], La2[2]));
        }
        else
        {
            sommets = new List<Vector3>();
            La2.ForEach(x=> sommets.AddRange(x.GetAllPoints()));
            sommets = sommets.OrderByDescending(x => x.y).Where(x => CheckConvexPoint(x, La2, P.transform.position)).Distinct().ToList();
            
            while (sommets.Count > 0 | !sommets.All(x => CheckOutsideSupp(x, La2, sommets,P.transform.position)))
            {
                Debug.Log(sommets.Count);
                var s = sommets.Where(x => CheckOutsideSupp(x, La2, sommets,P.transform.position)).First();
                var areteIncident = La2.Where(x => x.Contains(s)).ToList();
                var a1 = areteIncident[0];
                var a2 = areteIncident[1];
                
                var sommetTriangle = a1.GetAllPoints();
                sommetTriangle.AddRange(a2.GetAllPoints());
                sommetTriangle = sommetTriangle.Distinct().ToList();
                sommetTriangle = MeshCreator.Instance().getConvexEnvelopJarvis(sommetTriangle);

                Triangle t = new Triangle(CreateLine(sommetTriangle[0], sommetTriangle[1]),
                    CreateLine(sommetTriangle[1], sommetTriangle[2]), CreateLine(sommetTriangle[2], sommetTriangle[0]));

                var a3 = t.GetAllArete().Where(x => !La2.Contains(x)).First();
                La2.Remove(a1);
                La2.Remove(a2);
                La2.Add(a3);
                T.Add(t);
                sommets = new List<Vector3>();
                La2.ForEach(x=> sommets.AddRange(x.GetAllPoints()));
                sommets = sommets.OrderByDescending(x => x.y).Where(x => CheckConvexPoint(x, La2, P.transform.position)).Distinct().ToList();

            }
            

        }
    }

    public bool CheckOutsideSupp(Vector3 s, List<Arete> La2, List<Vector3> sommets, Vector3 P)
    {
        List<Vector3> temp = new List<Vector3>(sommets);
        temp.Remove(s);
        if (temp.Count == 0)
        {
            return true;
        }
        var areteIncident = La2.Where(x => x.Contains(s)).ToList();
        if (areteIncident.Count < 2)
        {
            return false;
        }
        var a1 = areteIncident[0];
        var a2 = areteIncident[1];
       
        var sommetTriangle = a1.GetAllPoints();
        sommetTriangle.AddRange(a2.GetAllPoints());
        sommetTriangle = sommetTriangle.Distinct().ToList();
        sommetTriangle = MeshCreator.Instance().getConvexEnvelopJarvis(sommetTriangle);
        sommetTriangle = sommetTriangle.Distinct().ToList();
        
        
        Triangle t = new Triangle(new Arete(sommetTriangle[0], sommetTriangle[1]),
            new Arete(sommetTriangle[1], sommetTriangle[2]), new Arete(sommetTriangle[2], sommetTriangle[0]));

        return temp.All(x => !IsInsideCircle(t, x));

    }

    public bool CheckConvexPoint(Vector3 s, List<Arete> La2, Vector3 P)
    {
        var areteIncident = La2.Where(x => x.Contains(s)).ToList();
        if (areteIncident.Count < 2)
        {
            return false;
        }
        var a1 = areteIncident[0];
        var a2 = areteIncident[1];
       
        var sommetTriangle = a1.GetAllPoints();
        sommetTriangle.AddRange(a2.GetAllPoints());
        sommetTriangle = sommetTriangle.Distinct().ToList();
        sommetTriangle = MeshCreator.Instance().getConvexEnvelopJarvis(sommetTriangle);
        sommetTriangle = sommetTriangle.Distinct().ToList();
        
        var temp = sommetTriangle.Where(x => !x.Equals(s)).OrderByDescending(x => x.y).ToList();
        var angle = GetAngle(temp[0], s, temp[1]);
        if (isLeft(temp[0], s, P))
        {
            angle = 360 - angle;
        }
        return angle < 180;
    }

    public List<Arete> AreteSeenBy(Vector3 P)
    {
        List<Arete> L = new List<Arete>();
        List<GameObject> polygon = pointsList.ToList();
        List<GameObject> envelop = new List<GameObject>();
        envelop = getConvexEnvelopJarvis(polygon);
        for (int i = 0; i < envelop.Count-1; i++)
        {
            var left = isLeft(envelop[i].transform.position, envelop[(i+1) % envelop.Count].transform.position,
                P);
            if (left)
            {
                L.Add(CreateLine(envelop[i].transform.position, envelop[(i+1) % envelop.Count].transform.position));
            }
        }
        return L;
    }

    public bool IsInsideCircle(Arete a, Vector3 P)
    {
        List<Triangle> t = GetTriangleFromArete(a);
        if (t.Count == 0)
        {
            return false;
        }
        Vector3 center = GetCentreCercleCirconscrit(t[0]);
        float rayon = Mathf.Abs(Vector3.Distance(center, a.GetPointA()));

        return rayon > Vector3.Distance(center, P);
    }

    public bool IsInsideCircle(Triangle t, Vector3 P)
    {
        Vector3 center = GetCentreCercleCirconscrit(t);
        float rayon = Mathf.Abs(Vector3.Distance(center, t.GetSommet()[0]));

        return rayon > Vector3.Distance(center, P);
    }

    public Vector3 GetCentreCercleCirconscrit(Triangle t)
    {
        List<Vector3> sommets = t.GetSommet();
    
        Vector3 milieu = new Vector3((sommets[0][0] + sommets[1][0])/2, (sommets[0][1] + sommets[1][1])/2, 0);
        
        float a = -((sommets[1][0] - sommets[0][0])/(sommets[1][1] - sommets[0][1]));
        var b = - (a * milieu.x) + milieu.y;

        Vector3 milieu2 = new Vector3((sommets[1][0] + sommets[2][0])/2, (sommets[1][1] + sommets[2][1])/2, 0);
        
        float ap = -((sommets[2][0] - sommets[1][0])/(sommets[2][1] - sommets[1][1]));
        var bp = milieu2.y - (ap * milieu2.x);

        var x = (bp - b)/(a - ap);
        var y  = (a * x) + b;
        Vector3 centreCircleP = new Vector3(x, y, 0);
        
        return centreCircleP;
    }

    public GameObject GetLastPoint() {
        return pointsList[pointsList.Count - 1];
    }

    public List<Triangle> get3DConvexEnvelop(List<Vector3> points)
    {

        if (points.Count < 4)
        {
            return null;
        }

        List<Vector3> Cq = points.Take(4).ToList();
        List<Vector3> CQplus1 = new List<Vector3>();
        List<Vector3> pointToCheck = points.Skip(4).ToList();

        List<Triangle> TCq = new List<Triangle>();
        List<Arete> ACq = new List<Arete>();

        Triangle T1 = new Triangle(Cq[0], Cq[1], Cq[2]);
        Triangle T2 = new Triangle(Cq[0], Cq[2], Cq[3]);
        Triangle T3 = new Triangle(Cq[0], Cq[1], Cq[3]);
        Triangle T4 = new Triangle(Cq[1], Cq[2], Cq[3]);
        
        TCq.Add(T1);
        TCq.Add(T2);
        TCq.Add(T3);
        TCq.Add(T4);

        foreach (var triangle in TCq)
        {
            triangle.GetAllArete().ForEach(x => ACq.Add(new Arete(x.GetPointA(), x.GetPointB())));
        }

        ACq = ACq.Distinct().ToList();
        int q = 4;


        for (int i = 0; i < pointToCheck.Count; i++)
        {
            List<Triangle> faceBleu = new List<Triangle>();
            List<Triangle> faceRouge = new List<Triangle>();

            Debug.Log($"N° T = {TCq.Count}");
            Debug.Log($"N° A = {ACq.Count}");
            
            for (int j = 0; j < TCq.Count; j++)
            {
                var triangle = TCq[j];

                if (CheckSideOfAPlan(Cq, triangle, pointToCheck[i]))
                {
                    faceRouge.Add(triangle);
                }
                else
                {
                    faceBleu.Add(triangle);
                }
            }

            if (faceBleu.Count == 0)
            {
                CQplus1 = Cq;
            }
            else
            {
                List<Arete> aretesRouges = new List<Arete>();
                List<Arete> aretesBleues = new List<Arete>();
                List<Arete> aretesViolettes = new List<Arete>();
                List<Vector3> sommetsRouges = new List<Vector3>();
                List<Vector3> sommesBleus = new List<Vector3>();
                List<Vector3> sommetsViolets = new List<Vector3>();

                foreach (var a in ACq)
                {
                    if (faceRouge.Where(x => x.ContainsArete(a)).ToList().Count == 2)
                    {
                        aretesRouges.Add(a);
                        //a.GetLine().GetComponent<LineRenderer>().material = lineMatColorRed;
                    }else if (faceBleu.Where(x => x.ContainsArete(a)).ToList().Count == 2)
                    {
                        aretesBleues.Add(a);
                        //a.GetLine().GetComponent<LineRenderer>().material = lineMatColorBlue;
                    }
                    else
                    {
                        aretesViolettes.Add(a);
                        //a.GetLine().GetComponent<LineRenderer>().material = lineMatColorPurple;
                    }
                }


                foreach (var a in aretesBleues)
                {
                    sommesBleus.AddRange(a.GetAllPoints());
                }
                
                foreach (var a in aretesRouges)
                {
                    sommetsRouges.AddRange(a.GetAllPoints());
                }
                
                foreach (var a in aretesViolettes)
                {
                    sommetsViolets.AddRange(a.GetAllPoints());
                }

                TCq = TCq.Where(x => faceRouge.Contains(x)).ToList();
                CQplus1 = new List<Vector3>();
                ACq = new List<Arete>();
                CQplus1.AddRange(points.Where(x => sommetsRouges.Contains(x) || sommetsViolets.Contains(x)).ToList());
                CQplus1 = CQplus1.Distinct().ToList();
                CQplus1.Add(pointToCheck[i]);

                foreach (var a in aretesViolettes)
                {
                    Arete a1 = new Arete(a.GetPointA(), pointToCheck[i]);
                    Arete a2 = new Arete(a.GetPointB(), pointToCheck[i]);
                    ACq.Add(a1);
                    ACq.Add(a2);
                    ACq.Add(a);
                    
                    Triangle temp = new Triangle(a, a1, a2);
                    TCq.Add(temp);
                }
                
                foreach (var a in aretesRouges)
                {
                    ACq.Add(a);
                }

                ACq = ACq.Distinct().ToList();
                
                
            }

            Cq = CQplus1;
            

        }


        /*foreach (var a in ACq)
        {
            a.GetLine().GetComponent<LineRenderer>().material = lineMatColorPurple;
        }*/
        
        return TCq;
    }

    public bool CheckSideOfAPlan(List<Vector3> Cq, Triangle triangle, Vector3 P)
    {
        var x = Cq.Where(x => !triangle.GetSommet().Contains(x)).First();
        var sommets = triangle.GetSommet();
                
        var Bp = sommets[1] - sommets[0];
        var Cp = sommets[2] - sommets[0];
                
        //Coté de Cq par rapport a H
        var Xp = x - sommets[0];
        
        /*
         * B.x | C.x | X.x
         * B.y | C.y | X.y
         * B.z | C.z | X.z
         * 
         */

        var detCq = (Bp.x * Cp.y * Xp.z) + (Cp.x * Xp.y * Bp.z) + (Xp.x * Bp.y * Cp.z) - (Xp.x * Cp.y * Bp.z) - (Cp.x * Bp.y * Xp.z) -
                    (Bp.x * Xp.y * Cp.z);
        
                
                
        //Coté du pi par rapport a H
        x = P;
        Xp = x - sommets[0];
                
        var detPi = (Bp.x * Cp.y * Xp.z) + (Cp.x * Xp.y * Bp.z) + (Xp.x * Bp.y * Cp.z) - (Xp.x * Cp.y * Bp.z) - (Cp.x * Bp.y * Xp.z) -
                    (Bp.x * Xp.y * Cp.z);

        return detCq > 0 == detPi > 0;
    }

    public Vector3 get3DTriangleCenter(Triangle t)
    {
        List<Vector3> temp = t.GetSommet();

        var a = temp[0];
        var b = temp[1];
        var c = temp[2];

        var x = (a.x + b.x + c.x) / 3;
        var y = (a.y + b.y + c.y) / 3;
        var z = (a.z + b.z + c.z) / 3;

        return new Vector3(x, y, z);
    }
    
    public void AjoutDelauney3D(GameObject P)
    {
        if (pointsList.Count == 0)
        {
            pointsList.Add(P);
            return;
        }
        
        List<GameObject> pointSorted = pointsList.OrderBy(x => x.transform.position.x).ToList();
        List<GameObject> polygon = new List<GameObject>();

        if (pointsList.Count == 1 )
        {
            CreateLine(pointSorted[0].transform.position, P.transform.position);
        
            pointsList.Add(P);
            polygon.Add(pointSorted[0]);
            polygon.Add(P);
            pointSorted.RemoveAt(0);
            return;
        }
        Vector3 v = pointSorted[0].transform.position - pointSorted[1].transform.position;
        Vector3 u = pointSorted[1].transform.position - P.transform.position;
        if (isColinear(v,u))
        {
            GameObject nearest = pointSorted.OrderBy(x => Vector3.Distance(x.transform.position, P.transform.position))
                .First();
            CreateLine(nearest.transform.position, P.transform.position);
            pointsList.Add(P);
            return;
        }

        if (pointsList.Count == 2)
        {
            Arete a = CreateLine(pointSorted[0].transform.position, P.transform.position);
            Arete b = CreateLine(pointSorted[1].transform.position, P.transform.position);
            Arete c = CreateLine(pointSorted[0].transform.position, pointSorted[1].transform.position);

            Triangle t = new Triangle(a, b, c);
            pointsList.Add(P);
            T.Add(t);
            return;
        }

        if (pointsList.Count == 3)
        {
            for (int i = 0; i < 3; i++)
            {
                Triangle t = new Triangle(pointSorted[i].transform.position,
                    pointSorted[(i + 1) % 3].transform.position, P.transform.position);
                T.Add(t);
                
                foreach (var a in t.GetAllArete())
                {
                    CreateLine(a.GetPointA(), a.GetPointB());
                }
            }
            pointsList.Add(P);
            F.Add(new Tetraèdre(T[0], T[1], T[2], T[3]));
            return;
        }
        
        /*List<GameObject> envelop = new List<GameObject>();

        if (T.Count == 0)
        {
            for (int i = 0; i < pointSorted.Count-1; i++)
            {
                Arete areteA = CreateLine(pointSorted[i].transform.position, P.transform.position);
                Arete areteB = CreateLine(pointSorted[(i+1)].transform.position, P.transform.position);
                Arete areteC = CreateLine(pointSorted[i].transform.position,
                    pointSorted[(i + 1)].transform.position);
                Triangle t = new Triangle(areteA, areteB, areteC);
                T.Add(t);
            }
            pointsList.Add(P);
            return;
        }*/

        Tetraèdre insider = GetTetraContainingPoint(P.transform.position);

        //List<Arete> L = new List<Arete>();
        
        if (insider != null)
        {
            /*foreach (var t in insider.GetFaces())
            {
                List<Vector3> tetra = new List<Vector3>();
                insider.GetFaces().ForEach(x => tetra.AddRange(x.GetSommet()));
                tetra = tetra.Distinct().ToList();
                if (!CheckSideOfAPlan(tetra, t, P.transform.position))
                {
                    List<Arete> arete = t.GetAllArete().ToList();

                    L.AddRange(arete);
            
                    T.Remove(t);
                }
                
            }*/
            
            List<Vector3> tetra = new List<Vector3>();
            insider.GetFaces().ForEach(x => tetra.AddRange(x.GetSommet()));
            tetra = tetra.Distinct().ToList();

            foreach (var t in insider.GetFaces())
            {
                foreach (var a in t.GetAllArete())
                {
                    Arete a1 = CreateLine(a.GetPointA(), P.transform.position);
                    Arete a2 = CreateLine(a.GetPointB(), P.transform.position);

                    Triangle temp = new Triangle(a, a1, a2);
                    T.Add(temp);
                }
            }
            pointsList.Add(P);
            return;
            
        }

        insider = GetSphereTetraContainingPoint(P.transform.position);
        if (insider != null)
        {
            Triangle faceToPoint = null;
            foreach (var t in insider.GetFaces())
            {
                List<Vector3> tetra = new List<Vector3>();
                insider.GetFaces().ForEach(x => tetra.AddRange(x.GetSommet()));
                tetra = tetra.Distinct().ToList();
                if (!CheckSideOfAPlan(tetra, t, P.transform.position))
                {
                    faceToPoint = t;
                    break;
                }
            }
            
            
        }
        else
        {
            List<Vector3> sommetsInsiderAndP = new List<Vector3>();
            insider.GetFaces().ForEach(x => sommetsInsiderAndP.AddRange(x.GetSommet()));
            sommetsInsiderAndP = sommetsInsiderAndP.Distinct().ToList();
            sommetsInsiderAndP.Add(P.transform.position);
            List<Triangle> envCon = get3DConvexEnvelop(sommetsInsiderAndP);

            foreach (var t in envCon)
            {
                t.GetAllArete().ForEach(x => CreateLine(x.GetPointA(), x.GetPointB()));
                T.Add(t);
            }
        }
        
        /*while (L.Count > 0)
        {
            Arete a = L[0];
            if (IsInsideCircle(a,P.transform.position))
            {
                List<Triangle> list = GetTriangleFromArete(a);
                for (int i = 0; i < list.Count; i++)
                {
                    var sommetT = list[i].GetAllArete();
                    sommetT.Remove(a);
                    L.AddRange(sommetT);
                    Destroy(a.GetLine());
                    A.Remove(a);
                    T.Remove(list[i]);
                }
                
            }
            else
            {
                Arete areteA = CreateLine(a.GetPointA(), P.transform.position);
                Arete areteB = CreateLine(a.GetPointB(), P.transform.position);
                Triangle t = new Triangle(areteA, areteB, a);
                T.Add(t);
            }
            L.Remove(a);
        }*/
        pointsList.Add(P);

    }
    
    public Tetraèdre GetSphereTetraContainingPoint(Vector3 P)
    {
        foreach (var currentF in F)
        {
            //Instantiate(centerCircle, currentF.GetSphereCenter(), Quaternion.Euler(0, 0, 0));
            //Debug.Log(currentF.GetSphereCenter());
            if (currentF.ContainsPoint(P))
            {
                return currentF;
            }
        }
        
        return null;
    }

    public Tetraèdre GetTetraContainingPoint(Vector3 P)
    {
        foreach (var currentF in F)
        {
            //Instantiate(centerCircle, currentF.GetSphereCenter(), Quaternion.Euler(0, 0, 0));
            //Debug.Log(currentF.GetSphereCenter());
            bool isInside = true;
            foreach (var t in currentF.GetFaces())
            {
                List<Vector3> tetra = new List<Vector3>();
                currentF.GetFaces().ForEach(x => tetra.AddRange(x.GetSommet()));
                tetra = tetra.Distinct().ToList();
                if (!CheckSideOfAPlan(tetra, t, P))
                {
                    isInside = false;
                    break;
                }
            }
            if (isInside)
            {
                return currentF;
            }
        }
        
        return null;
    }

}


