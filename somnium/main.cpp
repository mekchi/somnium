#include "main.h"

/* fluid parameters */
#define TIME_STEP			0.003f
#define SMOOTH_LENGTH		0.01f
#define SPHERE_RADIUS		0.15f
#define INTERVAL			0.011f//0.02f
#define CELL_SIZE			0.02f
/* water */
#define WATER_REST_DENSITY	1000.0f
#define WATER_STIFFNESS		1.5f
#define WATER_VISCOSITY		0.2f
#define WATER_MASS			0.00020543f
#define WATER_RESTITUTION	0.1f//0.5f
//#define TIME_STEP			0.005f
//#define SMOOTH_LENGTH		0.01f
//#define SPHERE_RADIUS		0.01f
//#define INTERVAL			0.001f
//#define CELL_SIZE			0.006f

#define X_ROW				10
#define Y_ROW				10
#define Z_ROW				10
#define TOTAL_NUMBER		X_ROW * Y_ROW * Z_ROW

#define	SP_GRID_SIZE		((int)(2.0f * SPHERE_RADIUS / SMOOTH_LENGTH) + 2)
#define V_CELL_SIZE			0.007f
#define	V_GRID_SIZE			((int)(2.0f * SPHERE_RADIUS / V_CELL_SIZE) + 3)

static float tb_project_to_sphere(float x, float y)
{
    float d, t, z;

    d = sqrtf(x * x + y * y);
    if (d < tb_radius * 0.70710678118654752440f) /* Inside sphere */
	{    
        z = sqrtf(tb_radius * tb_radius - d * d);
    } 
	else /* On hyperbola */
	{           
        t = tb_radius / 1.41421356237309504880f;
        z = t * t / d;
    }
    
	return z;
}

static void track_ball(quat4f *q)
{
	vector3f axis;
	vector3f v;
    float phi, t;

	if (last_coor.x == curr_coor.x && last_coor.y == curr_coor.y) 
	{
        q_set_zero(q);

        return;
    }
	v_cross_product(&axis, &curr_coor, &last_coor);
	v_normalize_c(&axis);
	v_subtract_d(&v, &last_coor, &curr_coor);
	t = v_magnitude(&v) / (2.0f * tb_radius);
	if (t > 1.0f) t = 1.0f;
    if (t < -1.0f) t = -1.0f;
	phi = 2.0f * asinf(t);
	v_axis_to_quat(q, &axis, phi / 2.0f);
}

void init()
{
	/* mouse */
	v_set_zero(&last_coor);
	v_set_zero(&curr_coor);
	
	/* transformation */
	q_set_zero(&last_q_rotation);
	q_set_zero(&curr_q_rotation);
	q_set_zero(&curr_q_ball_rotation);
	q_set_zero(&last_q_ball_rotation);
	m_set_identity(m_rotation); 
	m_set_identity(m_ball_rotation); 

	/* sph */
	v_set(&gravity, 0.0f, -9.8f, 0.0f);
	properties.rest_density = WATER_REST_DENSITY;
	properties.restitution = WATER_RESTITUTION;
	properties.stiffness = WATER_STIFFNESS;
	properties.smooth_length = SMOOTH_LENGTH;
	properties.timestep = TIME_STEP;
	properties.viscosity = WATER_VISCOSITY;
	properties.mass = WATER_MASS;
	sph_create(&machine, TOTAL_NUMBER, &properties, 25, SP_GRID_SIZE);
	sph_set_gravity(&gravity);
	sph_set_current_sph(&machine);
	sphere_radius = SPHERE_RADIUS;
	v_set_zero(&sphere_center);
	sph_set_position(INTERVAL, X_ROW, Y_ROW, Z_ROW);
	sph_set_collision_object(&sphere_center, sphere_radius);


	mc_create(&data, V_GRID_SIZE, V_CELL_SIZE);
}

void deinit()
{
	sph_set_current_sph(&machine);
	sph_destroy(&machine);	

	mc_destroy(&data);
}

void check_cg_error(const char *s)
{
	CGerror error;
	const char *string = cgGetLastErrorString(&error);

	if (error != CG_NO_ERROR) 
	{
		cout << "testg" << ": " << s << ": " << string << endl;
		if (error == CG_COMPILER_ERROR) 
		{
			cout << cgGetLastListing(cg_context) << endl;
		}
		exit(1);
	}
}

void init_cg()
{
	cg_context = cgCreateContext();
	check_cg_error("creating context");
	cgGLSetDebugMode(CG_FALSE);
	cgSetParameterSettingMode(cg_context, CG_DEFERRED_PARAMETER_SETTING);

	cg_vprof = cgGLGetLatestProfile(CG_GL_VERTEX);
	cgGLSetOptimalOptions(cg_vprof);
	check_cg_error("selecting vertex profile");

	cg_vp = cgCreateProgramFromFile(cg_context, CG_SOURCE, vp_filename,
		cg_vprof, cg_func_name, NULL);
	check_cg_error("creating vertex program from file");
	cgGLLoadProgram(cg_vp);
	check_cg_error("loading vertex program");

	cg_fprof = cgGLGetLatestProfile(CG_GL_FRAGMENT);
	cgGLSetOptimalOptions(cg_fprof);
	check_cg_error("selecting fragment profile");

	cg_fp = cgCreateProgramFromFile(cg_context, CG_SOURCE, fp_filename,
		cg_fprof, cg_func_name, NULL);
	check_cg_error("creating fragment program from file");
	cgGLLoadProgram(cg_fp);
	check_cg_error("loading fragment program");

	//cg_vprm_mvp = cgGetNamedParameter(cg_fp, "tex0");

	//cg_fprm_bk_tex = cgGetNamedParameter(cg_fp, "tex0");
	//check_cg_error("getting tex0 parameter");
	//cgGLSetTextureParameter(cg_fprm_bk_tex, texture_buffer);
	//check_cg_error("setting tex0 2D texture");
	//cg_fprm_bump_tex = cgGetNamedParameter(cg_fp, "tex1");
	//check_cg_error("getting tex1 parameter");
	//cgGLSetTextureParameter(cg_fprm_bump_tex, bump_texture);
	//check_cg_error("setting tex1 2D texture");
}

char* load_bmp_image(const char *filename, int *width, int *height);

void load_texture()
{
	int w, h;
	char *data;

	glGenTextures(1, &skybox);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skybox);
	for (int i = 0; i < 6; i++) 
	{
		data = load_bmp_image(skybox_filname[i], &w, &h);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, 3,
			w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		free(data);
	}
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

void init_open_gl()
{		
	if (glewInit() != GLEW_OK)
	{
		cout << "Could not initialize GLEW" << endl;
		exit(1);
	}
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);			// Black Background
	glShadeModel(GL_SMOOTH);						// Enable Smooth Shading
	glClearDepth(1.0f);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	//glPixelStorei(GL_UNPACK_ALIGNMENT, 1); /* Tightly packed texture data. */

	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_GEN_R);

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	load_texture();
	init_cg();
}

void deinit_open_gl()
{
	glDeleteTextures(1, &skybox);

	cgDestroyProgram(cg_vp);
	cgDestroyProgram(cg_fp);
	cgDestroyContext(cg_context);
}

void update()
{
	/* smooth particle hydrodynamics */

	if (pause)
	{
		sph_update();
	}
	mc_metaballs(&machine.grid, &data);
}

inline void draw_skybox()
{
	glDisable(GL_TEXTURE_2D);
	glPushMatrix();
	glLoadIdentity();
	//glScalef(100.0f, 100.0f, 100.0f);
	glMultMatrixf(m_rotation);
	glScalef(50.0f, 50.0f, 50.0f);
	glEnable(GL_TEXTURE_CUBE_MAP);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skybox);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glBegin(GL_QUADS);
	for (int i = 0; i < 24; i++) 
	{
		glTexCoord3fv(cube_coords[i]);
		glVertex3fv(cube_coords[i]);
	}
	glEnd();	
	glPopMatrix();
	glDisable(GL_TEXTURE_CUBE_MAP);
	glEnable(GL_TEXTURE_2D);
}

inline void draw_sph()
{
	cgGLBindProgram(cg_vp);
	cgGLEnableProfile(cg_vprof);
	cgGLBindProgram(cg_fp);
	cgGLEnableProfile(cg_fprof);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skybox);
	r_draw_marching_cubes(&data, 800.0f);
	glBindTexture(GL_TEXTURE_2D, 0);

	cgGLDisableProfile(cg_vprof);
	cgGLDisableProfile(cg_fprof);
}
 
void draw()
{
	matrix44f m;
	quat4f q;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);			// Clear The Screen And The Depth Buffer
	glLoadIdentity();
	
	glTranslatef(0.0f, 0.0f, -10.0f);//-0.4f);
	//glScalef(100.0f, 100.0f, 100.0f);
	glScalef(35.0f, 35.0f, 35.0f);
	if (tb_active)
	{
		if (shift_down)
		{
			track_ball(&q);
			q_multiply_d(&curr_q_rotation, &last_q_rotation, &q);
			q_to_matrix(m_rotation, &curr_q_rotation);
		}
		else
		{
			track_ball(&q);
			q_multiply_d(&curr_q_ball_rotation, &last_q_ball_rotation, &q);
			q_to_matrix(m_ball_rotation, &curr_q_ball_rotation);

			//m_multiplication_mv(&gravity, m_rotaion);
			//sph_set_gravity(&gravity);
			sph_change_position(m_ball_rotation);
		}		
	}
	//glMultMatrixf(m_ball_rotation);

	update();

	draw_skybox();

	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
		glLoadIdentity();
		m_invert_d(m, m_rotation);
		glMultMatrixf(m);
		//glMultMatrixf(m_rotation);
		draw_sph();
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	//draw_sph();
	//glPushMatrix();
	//glLoadIdentity();
	//glTranslatef(0.0f, 0.0f, -20.0f);//-0.4f);
	////glScalef(100.0f, 100.0f, 100.0f);
	//glScalef(50.0f, 50.0f, 50.0f);
	//r_draw_points(&data);
	//glPopMatrix();
	
	//r_draw_marching_cubes(&data, 800.0f);
}

void change_open_gl_window_size(int width, int height)
{
	w_width		= (float)width;
	w_height	= (float)height;
	glViewport(0, 0, width, height);	
	glMatrixMode(GL_PROJECTION);						
	glLoadIdentity();			
	gluPerspective(FRUSTRUM_ANGLE, w_width / w_height, FRUSTRUM_NEAR, FRUSTRUM_FAR);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// review this key
void SetFullScreenMode(int width, int height)
{
	DEVMODE dm;

	memset(&dm, 0, sizeof(dm)); // try to find another way to clear dm
	dm.dmSize = sizeof(dm);
	dm.dmPelsWidth = width;
	dm.dmPelsHeight = height;
	dm.dmBitsPerPel = 32;
	dm.dmFields = DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;
		
	if (ChangeDisplaySettings(&dm, CDS_FULLSCREEN)!= DISP_CHANGE_SUCCESSFUL)
	{
		MessageBox(NULL, "The Requested Fullscreen Mode Is Not Supported", "Error", MB_OK);
		fullscreen = false;
	}
}

LONG WINAPI WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch(uMsg)
	{
		case WM_PAINT:
			break;
		case WM_SIZE:
			change_open_gl_window_size(LOWORD(lParam), HIWORD(lParam));
			PostMessage(hWnd, WM_PAINT, 0, 0);
			break;
		case WM_KEYDOWN:
			if (wParam == VK_SHIFT && !shift_down)
			{				
				shift_down = 1;
				last_coor.x = 2.0f * (float)LOWORD(lParam) / w_width - 1.0f;
				last_coor.y = 1.0f - 2.0f * (float)HIWORD(lParam) / w_height;
				last_coor.z = tb_project_to_sphere(last_coor.x, last_coor.y);
				curr_coor = last_coor;
			}
			break;
		case WM_KEYUP:
			if (wParam == VK_SHIFT)
			{
				shift_down = 0;
			}
			break;
		case WM_MOUSEMOVE:
			if (tb_active)
			{
				curr_coor.x = 2.0f * (float)LOWORD(lParam) / w_width - 1.0f;
				curr_coor.y = 1.0f - 2.0f * (float)HIWORD(lParam) / w_height;
				curr_coor.z = tb_project_to_sphere(curr_coor.x, curr_coor.y);
			}
			break;
		case WM_LBUTTONDOWN:
			tb_active = 1;
			last_coor.x = 2.0f * (float)LOWORD(lParam) / w_width - 1.0f;
			last_coor.y = 1.0f - 2.0f * (float)HIWORD(lParam) / w_height;
			last_coor.z = tb_project_to_sphere(last_coor.x, last_coor.y);
			curr_coor = last_coor;
			break;
		case WM_LBUTTONUP:
			tb_active = 0;
			if (shift_down)
				last_q_rotation = curr_q_rotation; 
			else
				last_q_ball_rotation = curr_q_ball_rotation;
			
			break;
		case WM_CHAR:
			switch (wParam)
			{
				case 27: // ESC key
					PostQuitMessage(0);
					break;
				case 32:
					if (pause)
						pause =	0;
					else
						pause = 1;
					break;
				case 115:
					if (shift_down)
					{
						
						shift_down = 0;
					}
					else
						shift_down = 1;
					break;
			}	
			break;
		case WM_CLOSE:
			PostQuitMessage(0);
			break;
		default:
			return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
	return 0;
}

void create_console()
{
	int hConHandle;
	long lStdHandle;
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	FILE *fp;

	AllocConsole();
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);
	coninfo.dwSize.Y = MAX_CONSOLE_LINES;
	SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);
	lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "w" );
	*stdout = *fp;
	setvbuf( stdout, NULL, _IONBF, 0 );
	lStdHandle = (long)GetStdHandle(STD_INPUT_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "r" );
	*stdin = *fp;
	setvbuf( stdin, NULL, _IONBF, 0 );
	lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "w" );
	*stderr = *fp;
	setvbuf( stderr, NULL, _IONBF, 0 );
	ios::sync_with_stdio();
}

bool create_window(TCHAR* title, int x, int y, int width, int height, BYTE type, DWORD flags)
{
    int         pf;
    WNDCLASS    wc;
    PIXELFORMATDESCRIPTOR pfd;
	DWORD		dwExStyle;				
	DWORD		dwStyle;

	RECT WindowRect;							// Grabs Rectangle Upper Left / Lower Right Values
	WindowRect.left		=(long)0;						// Set Left Value To 0
	WindowRect.right	=(long)width;						// Set Right Value To Requested Width
	WindowRect.top		=(long)0;							// Set Top Value To 0
	WindowRect.bottom	=(long)height;	

    /* only register the window class once - use hInstance as a flag. */
    if (!hInstance) 
	{
		hInstance		= GetModuleHandle(NULL);
		wc.style		= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
		wc.lpfnWndProc	= (WNDPROC)WindowProc;
		wc.cbClsExtra	= 0;
		wc.cbWndExtra	= 0;
		wc.hInstance	= hInstance;
		wc.hIcon		= LoadIcon(NULL, IDI_WINLOGO);
		wc.hCursor		= LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground	= NULL;
		wc.lpszMenuName	= NULL;
		wc.lpszClassName	= "somnium";

		if (!RegisterClass(&wc)) 
		{
			MessageBox(NULL, "RegisterClass() failed:  \
				Cannot register window class.", "Error", MB_OK);
			return false;
		}
	}
	//more study
	//ShowCursor(FALSE);	
	if (fullscreen)
	{
		SetFullScreenMode(width, height);
		dwExStyle = WS_EX_APPWINDOW;					// Window Extended Style
		dwStyle = WS_POPUP;						// Windows Style
		//ShowCursor(FALSE);	
	}
	else
	{
		dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;			// Window Extended Style
		dwStyle = WS_OVERLAPPEDWINDOW;	
	}
	AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);
	hWnd = CreateWindowEx(dwExStyle, "somnium", title
		, dwStyle| WS_CLIPSIBLINGS | WS_CLIPCHILDREN
		, x, y, WindowRect.right-WindowRect.left
		, WindowRect.bottom-WindowRect.top, NULL, NULL, hInstance, NULL);
    if (hWnd == NULL) 
	{
		MessageBox(NULL, "CreateWindow() failed:  Cannot create a window.", "Error", MB_OK);
		return false;
    }
    hDC = GetDC(hWnd);
    memset(&pfd, 0, sizeof(pfd));
    pfd.nSize        = sizeof(pfd);
    pfd.nVersion     = 1;
    pfd.dwFlags      = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | flags;
    pfd.iPixelType   = type;
    pfd.cColorBits   = 32;
	pfd.dwLayerMask = PFD_MAIN_PLANE;

    pf = ChoosePixelFormat(hDC, &pfd);
    if (pf == 0) 
	{
		MessageBox(NULL, "ChoosePixelFormat() failed:  \
		   Cannot find a suitable pixel format.", "Error", MB_OK); 
		return false;
    } 
 
    if (SetPixelFormat(hDC, pf, &pfd) == FALSE) 
	{
		MessageBox(NULL, "SetPixelFormat() failed:  \
		   Cannot set format specified.", "Error", MB_OK);
		return false;
    } 

    DescribePixelFormat(hDC, pf, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

    //ReleaseDC(hWnd, hDC);

    return true;
}    

int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst,
	LPSTR lpszCmdLine, int nCmdShow)
{
    HGLRC hRC;		/* opengl context */
    MSG   msg;		/* message */

	hInstance	= 0;
	hDC			= NULL;
	hWnd		= NULL;
	fullscreen	= false;
	w_width		= w_width;
	w_height	= w_height;
	tb_active	= 0;
	tb_radius	= 1.0f;
	shift_down	= 0;
	pause		= 0;

	create_console();
	if (!create_window("somnium", 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, PFD_TYPE_RGBA, PFD_DOUBLEBUFFER))
		exit(1);
    hRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hRC);

    ShowWindow(hWnd, SW_SHOW);
	SetForegroundWindow(hWnd); //?
	SetFocus(hWnd);
	change_open_gl_window_size(WINDOW_WIDTH, WINDOW_HEIGHT); // must be called before initGL
	
	init_open_gl();
	init();
	
	// check performance with extended conditions in the main massage loop
	BOOL b = 1; // error handling style

    while((b = GetMessage(&msg, NULL, 0, 0)) != 0) 
	{
		static unsigned int last_time, curr_time, delta = 0;
		static int k = 0;
		
		if (b == -1)
		{
			MessageBox(NULL, "Error occured in the main loop", "Error", MB_OK);
		}
		else
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		last_time = GetTickCount();
		draw();
		SwapBuffers(hDC);
		curr_time = GetTickCount();
		delta += curr_time - last_time;
		if (delta > 1000)
		{
			delta = 0;
			fps += (float)frame_count;
			frame_count = 0;
			k++;
			if (k > 10)
			{
				fps /= 10.0f;
				cout << fps << endl;
				k = 0;
				fps = 0.0f;
			}
		}
		frame_count++;
    }

	deinit_open_gl();
	deinit();

	//ChangeDisplaySettings(NULL, 0);					// If So Switch Back To The Desktop
	//ShowCursor(TRUE);
    wglMakeCurrent(NULL, NULL);
    ReleaseDC(hWnd, hDC);
    wglDeleteContext(hRC);
    DestroyWindow(hWnd);
	UnregisterClass("somnium", hInstance);

    return msg.wParam;
}

static unsigned int getint(FILE *fp)
{
	int c, c1, c2, c3;

	/*  get 4 bytes */ 
	c = getc(fp);  
	c1 = getc(fp);  
	c2 = getc(fp);  
	c3 = getc(fp);

	return ((unsigned int) c) +   
		(((unsigned int) c1) << 8) + 
		(((unsigned int) c2) << 16) +
		(((unsigned int) c3) << 24);
}

static unsigned int getshort(FILE *fp)
{
	int c, c1;

	/* get 2 bytes*/
	c = getc(fp);  
	c1 = getc(fp);

	return ((unsigned int) c) + (((unsigned int) c1) << 8);
}

char* load_bmp_image(const char *filename, int *width, int *height) 
{
    FILE *file;
    unsigned long size;                 /*  size of the image in bytes. */
    unsigned long i;                    /*  standard counter. */
    unsigned short int planes;          /*  number of planes in image (must be 1)  */
    unsigned short int bpp;             /*  number of bits per pixel (must be 24) */
    char temp;                          /*  used to convert bgr to rgb color. */
	char *data;

    /*  make sure the file is there. */
    if ((file = fopen(filename, "rb"))==NULL) {
      printf("File Not Found : %s\n",filename);
      return NULL;
    }
    
    /*  seek through the bmp header, up to the width height: */
    fseek(file, 18, SEEK_CUR);

    /*  No 100% errorchecking anymore!!! */

    /*  read the width */    
	(*width) = getint (file);
    
    /*  read the height */ 
    (*height) = getint (file);
    
    /*  calculate the size (assuming 24 bits or 3 bytes per pixel). */
    size = (*width) * (*height) * 3;

    /*  read the planes */    
    planes = getshort(file);
    if (planes != 1) {
	printf("Planes from %s is not 1: %u\n", filename, planes);
	return NULL;
    }

    /*  read the bpp */    
    bpp = getshort(file);
    if (bpp != 24) {
      printf("Bpp from %s is not 24: %u\n", filename, bpp);
      return NULL;
    }
	
    /*  seek past the rest of the bitmap header. */
    fseek(file, 24, SEEK_CUR);

    /*  read the data.  */
    data = (char*)malloc(size);
    if (data == NULL) {
	printf("Error allocating memory for color-corrected image data");
	return NULL;	
    }

    if ((i = fread(data, size, 1, file)) != 1) {
	printf("Error reading image data from %s.\n", filename);
	return NULL;
    }

    for (i=0;i<size;i+=3) { /*  reverse all of the colors. (bgr -> rgb) */
      temp = data[i];
      data[i] = data[i+2];
      data[i+2] = temp;
    }

    fclose(file);

    return data;
}