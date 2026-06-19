varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;

uniform sampler2DShadow shadowMap;
uniform sampler2D diffuseMap;
uniform sampler2D normalMap;

// Gamma correction toggle
const float GAMMA = 2.2;

float attenuation(vec3 p, int i)
{
    vec4 p_l = gl_LightSource[i].position;
    if (p_l.w == 0.0) return 1.0;
    float d = distance(p, p_l.xyz);
    float k0 = gl_LightSource[i].constantAttenuation;
    float k1 = gl_LightSource[i].linearAttenuation;
    float k2 = gl_LightSource[i].quadraticAttenuation;
    return 1.0 / (k0 + k1*d + k2*d*d);
}

float spotlight(vec3 p, int i)
{
    if (gl_LightSource[i].spotCosCutoff < 0.0) return 1.0;
    vec4 p_l = gl_LightSource[i].position;
    if (p_l.w == 0.0) return 1.0;
    vec3 v = normalize(p - p_l.xyz);
    vec3 s = normalize(gl_LightSource[i].spotDirection);
    float cosine = max(dot(v, s), 0.0);
    float cutOffOuter = gl_LightSource[i].spotCosCutoff;
    float epsilon = 0.05; // softer edge falloff
    float intensity = smoothstep(cutOffOuter - epsilon, cutOffOuter, cosine);
    return pow(intensity, gl_LightSource[i].spotExponent);
}

vec3 applyRimLighting(vec3 normal, vec3 viewDir, vec3 rimColor, float rimPower)
{
    float rim = 1.0 - max(dot(normal, viewDir), 0.0);
    rim = pow(rim, rimPower);
    return rimColor * rim;
}

vec3 gammaCorrect(vec3 color)
{
    return pow(color, vec3(1.0 / GAMMA));
}

vec4 shade(vec3 p, vec3 v, vec3 n)
{
    vec3 Ie = gl_FrontMaterial.emission.rgb;
    vec3 Ia = gl_FrontLightModelProduct.sceneColor.rgb;
    vec3 Il = vec3(0.0);
    vec3 texColor = texture(diffuseMap, vTexCoord).rgb;

    for (int i = 0; i < gl_MaxLights; ++i)
    {
        vec4 p_l = gl_LightSource[i].position;
        vec3 l = normalize(p_l.xyz - p * p_l.w);
        vec3 h = normalize(l + v);

        float s_m = gl_FrontMaterial.shininess;
        float cosNL = max(dot(n, l), 0.0);
        float cosNH = max(dot(n, h), 0.0);

        float att = attenuation(p, i);
        float spot = spotlight(p, i);

        vec3 Iambient  = gl_FrontLightProduct[i].ambient.rgb * texColor;
        vec3 Idiffuse  = gl_FrontLightProduct[i].diffuse.rgb * cosNL * texColor;
        vec3 Ispecular = gl_FrontLightProduct[i].specular.rgb * pow(cosNH, s_m);

        // Add soft cool/warm lighting tint
        Iambient  *= 1.2;               // boost ambient a bit
        Idiffuse  *= vec3(1.0, 0.95, 0.9); // warm tint
        Ispecular *= vec3(0.8, 0.9, 1.0);  // cool tint

        vec3 phong = (Iambient + Idiffuse + Ispecular) * att * spot;
        Il += phong;
    }

    // Optional rim lighting
    vec3 rimColor = vec3(1.0); // white rim
    Il += applyRimLighting(n, v, rimColor, 4.0); // rimPower ~4–6

    vec3 finalColor = clamp(Ie + Ia + Il, 0.0, 1.0);
    finalColor = gammaCorrect(finalColor);

    return vec4(finalColor, gl_FrontMaterial.diffuse.a);
}

void main(void)
{
    vec3 view = normalize(-vPosition.xyz);
    vec3 normal = normalize(vNormal);
    vec4 shaded = shade(vPosition.xyz, view, normal);

    // Apply soft shadow
    float shadow = shadow2DProj(shadowMap, gl_TexCoord[1]).a;
    // shaded.rgb *= shadow;

    gl_FragColor = vec4(shaded.rgb, shadow);
}