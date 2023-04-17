#include "DualContouring.h"

#include <iostream>

void DualContouring::execute()
{
    createVertices();
    createFaces();
    fillBuffers();
}

std::tuple<std::vector<GLfloat>, std::vector<GLfloat>, std::vector<unsigned int>> DualContouring::getData()
{   
    return std::make_tuple(_vertexBuffer, _colorBuffer, _triangles);
}

double DualContouring::sdf(Eigen::Vector3d pos)
{
    double x = pos.x() - _dimensions.x() / 2.0;
    double y = pos.y() - _dimensions.y() / 2.0;
    double z = pos.z() - _dimensions.z() / 2.0;
    double result = 30.0f - sqrt(x*x+y*y+z*z);
    //double result = z;
    return result;
}

Eigen::Vector3d DualContouring::getNormalFromGradient(Eigen::Vector3d pos, Eigen::Vector3d& normal)
{
    const double c = 0.1;
    Eigen::Vector3d offset;

    offset = { c,0,0 };
    normal.x() = (sdf(pos + offset) - sdf(pos - offset)) / (2 * c);
    offset = { 0,c,0 };
    normal.y() = (sdf(pos + offset) - sdf(pos - offset)) / (2 * c);
    offset = { 0,0,c };
    normal.z() = (sdf(pos + offset) - sdf(pos - offset)) / (2 * c);
    normal.normalize();
    return normal;
}

unsigned int DualContouring::toMapIndex(Eigen::Vector3i voxelIndex)
{
    return voxelIndex.x() * _dimensions.y() * _dimensions.z()
        + voxelIndex.y() * _dimensions.z() 
        + voxelIndex.z();
}

float DualContouring::findZero(float a, float b)
{
    return (0.0f - a)/(b - a);
}

void DualContouring::fillBuffers()
{
    _vertexBuffer.reserve(_vertices.size() * sizeof(GLfloat) * 3);
    _colorBuffer.reserve(_vertices.size() * sizeof(GLfloat) * 3);

    for (auto vertex : _vertices)
    {
        _vertexBuffer.push_back(vertex.x() - _dimensions.x() / 2);
        _vertexBuffer.push_back(vertex.y() - _dimensions.y() / 2);
        _vertexBuffer.push_back(vertex.z() - _dimensions.z() / 2);

        _colorBuffer.push_back(vertex.x() / _dimensions.x() * 2);
        _colorBuffer.push_back(vertex.y() / _dimensions.y() * 2);
        _colorBuffer.push_back(vertex.z() / _dimensions.z() * 2);
    }
}

void DualContouring::createVertices()
{
    Eigen::Vector3d vertex = Eigen::Vector3d::Zero();
    //auto vertex = std::make_unique<Eigen::Vector3d>();
    volatile bool isValid = false;
    for (int x = 0; x < _dimensions.x(); ++x)
    {
        for (int y = 0; y < _dimensions.y(); ++y)
        {
            for (int z = 0; z < _dimensions.z(); ++z)
            {   
                Eigen::Vector3i voxelIndex = Eigen::Vector3i(x, y, z);
                findVertexInVoxel(voxelIndex, isValid, vertex);

                if (isValid)
                {
                    _vertexIndices.insert(std::make_pair(toMapIndex(voxelIndex), unsigned int(_vertices.size())));
                    _vertices.push_back(vertex);
                }
            }
        }
    }
}               
                //if (vertex.x() < 1)
                //    std::cout << vertex.x() << std::endl;
                //const Eigen::Vector3d vertex = std::get<1>(buffer);
                    // - _dimensions.x() / 2.0f << " " << vertex.y() - _dimensions.y() / 2.0f << " " << vertex.z() - _dimensions.z() / 2.0f << std::endl;
                //std::cout << "index: " << toMapIndex(voxelIndex) << " position: "
                //  << vertex.x() << " " << vertex.y() << " " << vertex.z() 
                //    << " Vertex index: " << int(_vertices.size()) << std::endl;

void DualContouring::createFaces()
{   
    Eigen::Vector3i voxelIndex;
    std::tuple<bool, bool, bool> zeroCrossings;
    bool isInside;

    for (int x = 0; x < _dimensions.x(); ++x)
    {
        for (int y = 0; y < _dimensions.y(); ++y)
        {
            for (int z = 0; z < _dimensions.z(); ++z)
            {
                voxelIndex = Eigen::Vector3i(x, y, z);
                zeroCrossings = findZeroCrossings(voxelIndex);
                isInside = sdf(voxelIndex.cast<double>()) > 0;
                std::string directions = "";

                if (std::get<0>(zeroCrossings))
                {
                    directions += "x";
                }
                if (std::get<1>(zeroCrossings))
                {
                    directions += "y";
                }
                if (std::get<2>(zeroCrossings))
                {
                    directions += "z";
                }

                if (directions == "")
                    continue;

                createFace(voxelIndex, directions, !isInside);
            }
        }
    }
}

void DualContouring::createFace(Eigen::Vector3i pos, std::string direction, bool clockwise)
{
    Eigen::Vector3i buffer = pos;
    if (direction.find('x') != std::string::npos && buffer.y() > 0 && buffer.z() > 0)
    {
        if (clockwise)
        {
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            //std::cout << "First point: " << buffer.x() << " " << buffer.y() << " " << buffer.z() << std::endl;
            buffer[2] -= 1; 
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            //std::cout << "Second point: " << buffer.x() << " " << buffer.y() << " " << buffer.z() << std::endl;
            buffer[2] += 1;
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            //std::cout << "Third point: " << buffer.x() << " " << buffer.y() << " " << buffer.z() << std::endl;
            buffer[1] += 1;
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            //std::cout << "Forth point: " << buffer.x() << " " << buffer.y() << " " << buffer.z() << std::endl;
        }
        else {
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] += 1;
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[2] += 1;
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
        }
    }

    buffer = pos;
    if (direction.find('y') != std::string::npos && buffer.x() > 0 && buffer.z() > 0)
    {
        if (clockwise)
        {
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] += 1;
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[2] += 1;
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
        }
        else {
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[2] += 1;
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] += 1;
            buffer[2] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
        }
    }

    buffer = pos;
    if (direction.find('z') != std::string::npos && buffer.x() > 0 && buffer.y() > 0)
    {
        if (clockwise)
        {
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] += 1;
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] += 1;
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
        }
        else {
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[0] += 1;
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] += 1;
            buffer[0] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
            buffer[1] -= 1;
            _triangles.push_back(_vertexIndices[toMapIndex(buffer)]);
        }
    }
}

void DualContouring::CreateVoxelFromPosition(Eigen::Vector3i voxelIndex, Voxel& voxel)
{

    Eigen::Vector3d pos = voxelIndex.cast<double>();

    Eigen::Vector3d offsetStart{ 0, 0, 0 };
    Eigen::Vector3d offsetEnd{ 0, 0, 0 };

    double startBuffer = 0;
    double endBuffer = 0;

    const double maxOffset = 0.999;

    // fill voxel data
    for (int y = 0; y <= 1; ++y)
    {
        for (int z = 0; z <= 1; ++z)
        {
            pos = voxelIndex.cast<double>();
            offsetStart = { 0, double(y), double(z) };
            offsetEnd = { 1.0, double(y), double(z) };
            //offsetStart *= maxOffset;
            //offsetEnd *= maxOffset;
            startBuffer = sdf(pos + offsetStart);
            endBuffer = sdf(pos + offsetEnd);

            if ((startBuffer > 0) != (endBuffer > 0))
            {
                double offsetToZero = findZero(startBuffer, endBuffer);
                //std::cout << "value start: " << startBuffer << std::endl;
                //std::cout << "value end: " << endBuffer << std::endl;
                //std::cout << "offset to zero: " << offsetToZero << std::endl;
                voxel.edges[y * 2 + z].point = pos + Eigen::Vector3d(offsetToZero, double(y), double(z));
                voxel.edges[y * 2 + z].valid = true;
                getNormalFromGradient(pos + Eigen::Vector3d(offsetToZero, double(y), double(z)), voxel.edges[y * 2 + z].normal);
            }

            // fill voxel corners and signs
            //voxel.signs[y * 4 + z * 2] = std::signbit(startBuffer) ? -1 : 1;
            //voxel.signs[y * 4 + z * 2 + 1] = std::signbit(endBuffer) ? -1 : 1;
            //voxel.corners[y * 4 + z * 2] = pos + offsetStart;
            //voxel.corners[y * 4 + z * 2 + 1] = pos + offsetEnd;
        }
    }

    for (int x = 0; x <= 1; ++x)
    {
        for (int z = 0; z <= 1; ++z)
        {
            pos = voxelIndex.cast<double>();
            offsetStart = { double(x), 0.0, double(z) };
            offsetEnd = { double(x), 1.0, double(z) };
            //offsetStart *= maxOffset;
            //offsetEnd *= maxOffset;
            startBuffer = sdf(pos + offsetStart);
            endBuffer = sdf(pos + offsetEnd);

            if ((startBuffer > 0) != (endBuffer > 0))
            {
                double offsetToZero = findZero(startBuffer, endBuffer);
                voxel.edges[4 + x * 2 + z].point = pos + Eigen::Vector3d(double(x), offsetToZero, double(z));
                voxel.edges[4 + x * 2 + z].valid = true;
                getNormalFromGradient(pos + Eigen::Vector3d(double(x), offsetToZero, double(z)), voxel.edges[4 + x * 2 + z].normal);
            }

            // fill voxel corners and signs
            //voxel.signs[y * 4 + z * 2] = std::signbit(startBuffer) ? -1 : 1;
            //voxel.signs[y * 4 + z * 2 + 1] = std::signbit(endBuffer) ? -1 : 1;
            //voxel.corners[y * 4 + z * 2] = pos + offsetStart;
            //voxel.corners[y * 4 + z * 2 + 1] = pos + offsetEnd;
            /*if ((sdf(pos + offsetStart) > 0) != (sdf(pos + offsetEnd) > 0))
                y = true;*/
        }
    }

    for (int x = 0; x <= 1; ++x)
    {
        for (int y = 0; y <= 1; ++y)
        {
            pos = voxelIndex.cast<double>();
            offsetStart = { double(x), double(y), 0.0 };
            offsetEnd = { double(x), double(y), 1.0 };
            //offsetStart *= maxOffset;
            //offsetEnd *= maxOffset;
            startBuffer = sdf(pos + offsetStart);
            endBuffer = sdf(pos + offsetEnd);

            if ((startBuffer > 0) != (endBuffer > 0))
            {
                double offsetToZero = findZero(startBuffer, endBuffer);
                voxel.edges[8 + x * 2 + y].point = pos + Eigen::Vector3d(double(x), double(y), offsetToZero);
                voxel.edges[8 + x * 2 + y].valid = true;
                getNormalFromGradient(pos + Eigen::Vector3d(double(x), double(y), offsetToZero), voxel.edges[8 + x * 2 + y].normal);
            }
           /* if ((sdf(pos + offsetStart) > 0) != (sdf(pos + offsetEnd) > 0))
                z = true;*/
        }
    }
}

std::tuple<bool, bool, bool> DualContouring::findZeroCrossings(Eigen::Vector3i voxelIndex)
{
    bool x = false, y = false, z = false;

    Eigen::Vector3d pos = voxelIndex.cast<double>();
    bool isInside = sdf(pos) > 0;

    Eigen::Vector3d offset;

    offset = { 1.0, 0, 0 };
    if ((sdf(pos + offset) > 0) != isInside)
        x = true;

    offset = { 0, 1.0, 0 };
    if ((sdf(pos + offset) > 0) != isInside)
        y = true;    

    offset = { 0, 0, 1.0 };
    if ((sdf(pos + offset) > 0) != isInside)
        z = true;

    return std::make_tuple(x,y,z);
}

void DualContouring::calculateQEF(std::vector<EdgeData> & edges, Eigen::MatrixXd & A, Eigen::VectorXd & b)
{
    // Initialize A and b to zero
    A.setZero();
    b.setZero();

    // Loop over all edges
    for (unsigned int i = 0; i < edges.size(); i++) {
        // Get the point and normal from the edge data
        const Eigen::Vector3d& p = edges[i].point;
        const Eigen::Vector3d& n = edges[i].normal;

        // A is simply the stacked normals
        A.row(i) = n;
        // 
        b[i] = p.x() * n.x() + p.y() * n.y() + p.z() * n.z();

        //std::cout << "Position: " << i << " :"
        //    << p.x() - _dimensions.x() / 2.0f << " "
        //    << p.y() - _dimensions.y() / 2.0f << " "
        //    << p.z() - _dimensions.z() / 2.0f << std::endl;
        //std::cout << "Position: " << i << " :"
        //    << p.x() << " "
        //    << p.y() << " "
        //    << p.z() << std::endl;
    }
    //std::cout << A << "\n  now b: \n" << b << std::endl;
}

void DualContouring::getForceOnPoint(std::vector<Eigen::Vector3d>& cornerForces, Eigen::Vector3i& voxelIndex , Eigen::Vector3d& position)
{

}

void DualContouring::findVertexInVoxel(Eigen::Vector3i& voxelIndex, volatile bool& success, Eigen::Vector3d& result)
{
    Voxel voxel;
    CreateVoxelFromPosition(voxelIndex, voxel);

    // find intersecting edges
    auto intersectingEdges = std::vector<EdgeData>();
    intersectingEdges.clear();
    //intersectingEdges.reserve(6);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int i = 0; i < 12; ++i)
    {
        if (voxel.edges[i].valid)
        {
            voxel.edges[i].normal.normalize();
            intersectingEdges.push_back(voxel.edges[i]);
            mean += voxel.edges[i].point;
        }
    }

    // if no intersecting edges, return false
    if (intersectingEdges.empty())
    {
        success = false;
        return;
    }


    Eigen::Vector3d meanNormal{ 0,0,0 };
    double similarity = 0;

    for (int i = 0; i < intersectingEdges.size(); ++i)
    {
        meanNormal += intersectingEdges[i].normal / intersectingEdges.size();
    }    
    for (int i = 0; i < intersectingEdges.size(); ++i)
    {
        similarity += intersectingEdges[i].normal.dot(meanNormal) / intersectingEdges.size();
    }

    Eigen::MatrixXd A(intersectingEdges.size(), 3);
    Eigen::VectorXd b(intersectingEdges.size());

    calculateQEF(intersectingEdges, A, b);

    // Solve the linear system Ax=b using Eigen's solver 
    result = A.fullPivHouseholderQr().solve(b);
    success = true;

    if (similarity >= 1 - 1e-5)
    {
        //std::cout << "Similarity: " << similarity << " Edges: " << intersectingEdges.size() << std::endl;
        //for (auto& edge : intersectingEdges)
        //{
        //    result += edge.point;
        //    std::cout << "Edgepoint: " << edge.point << std::endl;
        //}
        mean /= (intersectingEdges.size());
        result = mean;
        //std::cout << "Similarity: " << similarity << std::endl
        //    << "Edges: " << intersectingEdges.size() << std::endl
        //    << "Index: " << voxelIndex << std::endl
        //    << "Result: " << result << std::endl;
        success = true;
        //return;
    }


    //if ((result - voxelIndex.cast<double>()).norm() > 3)d
    //{
    //    for (auto& edge : intersectingEdges)
    //    {
    //        std::cout << "Normal: " << edge.normal << std::endl;
    //    }
    //    std::cout << "Number edges: " << intersectingEdges.size() << std::endl;
    //    //result = voxelIndex.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5);
    //    return;
    //}

    //auto& vertex  = result;
    //std::cout << vertex.x() - _dimensions.x() / 2.0f << " " << vertex.y() - _dimensions.y() / 2.0f << " " << vertex.z() - _dimensions.z() / 2.0f << std::endl;
    //std::cout << vertex.x() << " " << vertex.y() << " " << vertex.z() << std::endl;
    clipPosition(voxelIndex, result);

    //Eigen::Vector3d result = voxelIndex.cast<double>();
    ////Eigen::Vector3d result{ voxelIndex.x() + 0.5,  voxelIndex.y() + 0.5,  voxelIndex.z() + 0.5 };
    //if (x || y || z)
    //{
    //    //std::cout << result.x() << " " << result.y() << " " << result.z() << std::endl;
    //    return std::make_tuple(true, result);
    //}
}


void DualContouring::clipPosition(Eigen::Vector3i voxelIndex, Eigen::Vector3d& pos)
{
    if (voxelIndex.x() > (pos.x() + 0.5))
        pos.x() = voxelIndex.x();
    else if ((voxelIndex.x() + 1) < pos.x())
        pos.x() = voxelIndex.x() + 1;

    if (voxelIndex.y() > (pos.y() + 0.5))
        pos.y() = voxelIndex.y();
    else if ((voxelIndex.y() + 1) < pos.y())
        pos.y() = voxelIndex.y() + 1;

    if (voxelIndex.z() > (pos.z() + 0.5))
        pos.z() = voxelIndex.z();
    else if ((voxelIndex.z() + 1) < pos.z())
        pos.z() = voxelIndex.z() + 1;
}
