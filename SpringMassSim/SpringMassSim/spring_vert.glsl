
in vec3 vPos;
in vec3 vColor;

out vec3 fColor;

void main() {
	fColor = vColor;

	gl_Position = vec4(vPos, 1.0);
}