#ifndef CONTROLS_HPP
#define CONTROLS_HPP

void computeMatricesFromInputs(GLFWwindow* window);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

#endif