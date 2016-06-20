
in vec3 fPos;
in vec3 fCenter;
in float fRadius;
in vec3 fColor;

void main() {

	if (length(fPos - fCenter) <= fRadius) {
	    gl_FragColor = vec4(fColor, 1.0);
    } else {
        gl_FragColor = vec4(fColor, 0.0);
    }
}