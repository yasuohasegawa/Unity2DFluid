using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// reference: https://mikeash.com/pyblog/blog/images/fluid.c.file
// https://www.mikeash.com/pyblog/fluid-simulation-for-dummies.html

public class FluidDefines
{
    public static int IX(int x, int y, int N)
    {
        x = (int)Mathf.Clamp(x, 0, N - 1);
        y = (int)Mathf.Clamp(y, 0, N - 1);
        return x + y * N;
    }
}

public class FluidUtils
{
    public static void diffuse(int b, float[] x, float[] x0, float diff, float dt, int iter, int N)
    {
        float a = dt * diff * (N - 2) * (N - 2);
        lin_solve(b, x, x0, a, 1 + 6 * a, iter, N);
    }

    public static void lin_solve(int b, float[] x, float[] x0, float a, float c, int iter, int N)
    {
        float cRecip = 1.0f / c;
        for (int k = 0; k < iter; k++)
        {

            for (int j = 1; j < N - 1; j++)
            {
                for (int i = 1; i < N - 1; i++)
                {
                    x[FluidDefines.IX(i, j, N)] =
                        (x0[FluidDefines.IX(i, j, N)]
                            + a * (x[FluidDefines.IX(i + 1, j, N)]
                                    + x[FluidDefines.IX(i - 1, j, N)]
                                    + x[FluidDefines.IX(i, j + 1, N)]
                                    + x[FluidDefines.IX(i, j - 1, N)]
                                    + x[FluidDefines.IX(i, j, N)]
                                    + x[FluidDefines.IX(i, j, N)]
                            )) * cRecip;
                }
            }
            
            set_bnd(b, x, N);
        }
    }

    public static void project(float[] velocX, float[] velocY, float[] p, float[] div, int iter, int N)
    {
        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                div[FluidDefines.IX(i, j, N)] = -0.5f * (
                         velocX[FluidDefines.IX(i + 1, j, N)]
                        - velocX[FluidDefines.IX(i - 1, j, N)]
                        + velocY[FluidDefines.IX(i, j + 1, N)]
                        - velocY[FluidDefines.IX(i, j - 1, N)]
                    ) / N;
                p[FluidDefines.IX(i, j, N)] = 0;
            }
        }
        set_bnd(0, div, N);
        set_bnd(0, p, N);
        lin_solve(0, p, div, 1, 6, iter, N);

        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                velocX[FluidDefines.IX(i, j, N)] -= 0.5f * (p[FluidDefines.IX(i + 1, j, N)]
                                                - p[FluidDefines.IX(i - 1, j, N)]) * N;
                velocY[FluidDefines.IX(i, j, N)] -= 0.5f * (p[FluidDefines.IX(i, j + 1, N)]
                                                - p[FluidDefines.IX(i, j - 1, N)]) * N;
            }
        }
        set_bnd(1, velocX, N);
        set_bnd(2, velocY, N);
    }

    public static void advect(int b, float[] d, float[] d0, float[] velocX, float[] velocY, float dt, int N)
    {
        float i0, i1, j0, j1;

        float dtx = dt * (N - 2);
        float dty = dt * (N - 2);

        float s0, s1, t0, t1;
        float tmp1, tmp2, x, y;

        float Nfloat = N;
        float ifloat, jfloat;
        int i, j;

        for (j = 1, jfloat = 1; j < N - 1; j++, jfloat++)
        {
            for (i = 1, ifloat = 1; i < N - 1; i++, ifloat++)
            {
                tmp1 = dtx * velocX[FluidDefines.IX(i, j, N)];
                tmp2 = dty * velocY[FluidDefines.IX(i, j, N)];
                x = ifloat - tmp1;
                y = jfloat - tmp2;

                if (x < 0.5f) x = 0.5f;
                if (x > Nfloat + 0.5f) x = Nfloat + 0.5f;
                i0 = Mathf.Floor(x);
                i1 = i0 + 1.0f;
                if (y < 0.5f) y = 0.5f;
                if (y > Nfloat + 0.5f) y = Nfloat + 0.5f;
                j0 = Mathf.Floor(y);
                j1 = j0 + 1.0f;

                s1 = x - i0;
                s0 = 1.0f - s1;
                t1 = y - j0;
                t0 = 1.0f - t1;

                int i0i = (int)i0;
                int i1i = (int)i1;
                int j0i = (int)j0;
                int j1i = (int)j1;

                d[FluidDefines.IX(i, j, N)] =

                    s0 * (t0 * d0[FluidDefines.IX(i0i, j0i, N)]
                       + t1 * d0[FluidDefines.IX(i0i, j1i, N)])
                  + s1 * (t0 * d0[FluidDefines.IX(i1i, j0i, N)]
                       + t1 * d0[FluidDefines.IX(i1i, j1i, N)]);
            }
        }
        
        set_bnd(b, d, N);
    }

    public static void set_bnd(int b, float[] x, int N)
    {
        for (int i = 1; i < N - 1; i++)
        {
            x[FluidDefines.IX(i, 0, N)] = b == 2 ? -x[FluidDefines.IX(i, 1, N)] : x[FluidDefines.IX(i, 1, N)];
            x[FluidDefines.IX(i, N - 1, N)] = b == 2 ? -x[FluidDefines.IX(i, N - 2, N)] : x[FluidDefines.IX(i, N - 2, N)];
        }
        for (int j = 1; j < N - 1; j++)
        {
            x[FluidDefines.IX(0, j, N)] = b == 1 ? -x[FluidDefines.IX(1, j, N)] : x[FluidDefines.IX(1, j, N)];
            x[FluidDefines.IX(N - 1, j, N)] = b == 1 ? -x[FluidDefines.IX(N - 2, j, N)] : x[FluidDefines.IX(N - 2, j, N)];
        }

        x[FluidDefines.IX(0, 0, N)] = 0.5f * (x[FluidDefines.IX(1, 0, N)]
                                      + x[FluidDefines.IX(0, 1, N)]);
        x[FluidDefines.IX(0, N - 1, N)] = 0.5f * (x[FluidDefines.IX(1, N - 1, N)]
                                      + x[FluidDefines.IX(0, N - 2, N)]);

        x[FluidDefines.IX(N - 1, 0, N)] = 0.5f * (x[FluidDefines.IX(N - 2, 0, N)]
                                      + x[FluidDefines.IX(N - 1, 1, N)]);
        x[FluidDefines.IX(N - 1, N - 1, N)] = 0.5f * (x[FluidDefines.IX(N - 2, N - 1, N)]
                                      + x[FluidDefines.IX(N - 1, N - 2, N)]);
    }
}

// `pointer`* should be Array
public class Fluid
{
    public int size;
    float dt;
    float diff;
    float visc;

    float[] s; // previous density
    public float[] density;
         
    float[] Vx;
    float[] Vy;
         
    float[] Vx0; // previous
    float[] Vy0; // previous

    int iter = 4;

    public Fluid(int size, float dt, float diffusion, float viscosity)
    {
        int N = size;

        this.size = size;
        this.dt = dt;
        this.diff = diffusion;
        this.visc = viscosity;

        this.s = new float[N * N];
        this.density = new float[N * N];

        this.Vx = new float[N * N];
        this.Vy = new float[N * N];

        this.Vx0 = new float[N * N];
        this.Vy0 = new float[N * N];
    }

    public void AddDensity(int x, int y, float amount)
    {
        int N = this.size;
        this.density[FluidDefines.IX(x, y, N)] += amount;
    }

    public void AddVelocity(int x, int y, float amountX, float amountY)
    {
        int N = this.size;
        int index = FluidDefines.IX(x, y, N);

        this.Vx[index] += amountX;
        this.Vy[index] += amountY;
    }

    public void Step()
    {
        int N = this.size;
        float visc = this.visc;
        float diff = this.diff;
        float dt = this.dt;
        float[] Vx = this.Vx;
        float[] Vy = this.Vy;
        float[] Vx0 = this.Vx0;
        float[] Vy0 = this.Vy0;
        float[] s = this.s;
        float[] density = this.density;

        FluidUtils.diffuse(1, Vx0, Vx, visc, dt, iter, N);
        FluidUtils.diffuse(2, Vy0, Vy, visc, dt, iter, N);

        FluidUtils.project(Vx0, Vy0, Vx, Vy, iter, N);

        FluidUtils.advect(1, Vx, Vx0, Vx0, Vy0, dt, N);
        FluidUtils.advect(2, Vy, Vy0, Vx0, Vy0, dt, N);

        FluidUtils.project(Vx, Vy, Vx0, Vy0, iter, N);

        FluidUtils.diffuse(0, s, density, diff, dt, iter, N);
        FluidUtils.advect(0, density, s, Vx, Vy, dt, N);
    }
}

public class FluidController : MonoBehaviour
{
    [SerializeField]
    private GameObject m_prefab;

    private Fluid m_fluid;

    private float pmouseX;
    private float pmouseY;

    private MeshRenderer[,] m_meshRenders;

    private MaterialPropertyBlock m_materialPropertyBlock;

    // Start is called before the first frame update
    void Start()
    {
        m_materialPropertyBlock = new MaterialPropertyBlock();
        m_fluid = new Fluid(40, 0.05f, 0, 0);
        int N = m_fluid.size;
        float space = 0.5f;
        float cubeScale = 0.4f;
        m_meshRenders = new MeshRenderer[N, N];
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                float offset = (((N * 0.5f) * space) - (cubeScale*0.5f));
                float x = (i * space) - offset;
                float y = (j * space) - offset;

                var go = Instantiate(m_prefab) as GameObject;
                var pos = go.transform.position;
                pos.x = x;
                pos.y = y;
                pos.z = 0;

                var render = go.GetComponent<MeshRenderer>();
                go.transform.position = pos;

                m_meshRenders[i, j] = render;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        m_fluid.Step();
        Render();
        Fade();

        if (Input.GetMouseButton(0))
        {
            var mousePos = Input.mousePosition;

            float mouseX = (mousePos.x / Screen.width) * m_fluid.size;
            float mouseY = (mousePos.y / Screen.height) * m_fluid.size;

            float amX = mouseX - pmouseX;
            float amY = mouseY - pmouseY;

            int x = (int)mouseX;
            int y = (int)mouseY;
            //Debug.Log($"{x}:{y}");

            m_fluid.AddDensity(x, y, 100);
            m_fluid.AddVelocity(x, y, amX, amY);

            pmouseX = mouseX;
            pmouseY = mouseY;
        }
    }

    public void Render()
    {
        int N = m_fluid.size;
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                float x = i;
                float y = j;
                float d = m_fluid.density[FluidDefines.IX(i, j, N)];

                var col = new Color(d, d, d, d);

                m_meshRenders[i, j].GetPropertyBlock(m_materialPropertyBlock);
                m_materialPropertyBlock.SetColor("_Color", col);
                m_meshRenders[i, j].SetPropertyBlock(m_materialPropertyBlock);
            }
        }
    }

    public void Fade()
    {
        for (int i = 0; i< m_fluid.density.Length; i++)
        {
            float d = m_fluid.density[i];
            m_fluid.density[i] = Mathf.Clamp(d - 0.1f, 0, 255);
        }
    }
}
