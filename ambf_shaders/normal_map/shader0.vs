# version 120
attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTexCoord;
attribute vec4 aColor;
attribute vec3 aTangent;
attribute vec3 aBitangent;

varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;
varying vec4 lightPos[gl_MaxLights];
varying vec3 spotDir[gl_MaxLights];

void main(void)
{
   // pass along a transformed vertex position, normal, and texture
   vPosition = gl_ModelViewMatrix * gl_Vertex;
   gl_TexCoord[0] = gl_TextureMatrix[0] * vec4(aTexCoord, 1.0);

   float s = dot(gl_EyePlaneS[1], vPosition);
   float t = dot(gl_EyePlaneT[1], vPosition);
   float r = dot(gl_EyePlaneR[1], vPosition);
   float q = dot(gl_EyePlaneQ[1], vPosition);
   gl_TexCoord[1] = vec4(s, t, r, q);
   vTexCoord = aTexCoord;

  vec3 T = normalize(gl_NormalMatrix * aTangent);
  vec3 B = normalize(gl_NormalMatrix * aBitangent);
  vec3 N = gl_NormalMatrix * aNormal;

  // Orthogonalize T,B,N using the Grahm-Schmidt algorithm
  T = normalize(T - dot(T, N) * N);
  B = cross(N, T);

  mat3 TBN = transpose(mat3(T, B, N));

  for (int i = 0 ; i < gl_MaxLights; i++){
    vec3 p_l = TBN * vec3(gl_LightSource[i].position);
    lightPos[i] = vec4(p_l, gl_LightSource[i].position.w);
    spotDir[i] = TBN * gl_LightSource[i].spotDirection;
  }

  vPosition.xyz = TBN * vec3(vPosition);
  vPosition.w = vPosition.w;
  vNormal = TBN * N;

   gl_Position = ftransform();
}
