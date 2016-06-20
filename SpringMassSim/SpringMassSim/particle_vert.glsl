
in vec3 vPos;
in vec3 vCenter;
in vec3 vColor;
in float vRadius;

out vec3 fPos;
out vec3 fCenter;
out float fRadius;
out vec3 fColor;

void main() {
	fCenter = vCenter;
	fColor = vColor;
	fPos = vPos;
	fRadius = vRadius;

	gl_Position = vec4(vPos, 1.0);
}