/** dMlsfield Colliders
 * \file mlsfield.h
 * \author Yong-Ho Yoo
 * \brief mlsfield is based on the heightfield grid structure. 
 *        Heightfield triangles have been replaced by rectangulars and boxs. 
 *
 */

#ifndef _DMLSFIELD_H_
#define _DMLSFIELD_H_

//------------------------------------------------------------------------------

#include <ode/common.h>
//#include "collision_kernel.h"
#include <mars/sim/collision_kernel.h>


#define MLSFIELDMAXCONTACTPERCELL 4   // maximum contacts per object


struct dxMlsfieldData;
typedef struct dxMlsfieldData* dMlsfieldDataID;

typedef dReal dMlsfieldGetHeight( void* p_user_data, int x, int z );

ODE_API dGeomID dCreateMlsfield( dSpaceID space,
					dMlsfieldDataID data, int bPlaceable );

ODE_API dMlsfieldDataID dGeomMlsfieldDataCreate(void);

ODE_API void dGeomMlsfieldDataDestroy( dMlsfieldDataID d );

ODE_API void dGeomMlsfieldDataBuildCallback( dMlsfieldDataID d,
				void* pUserData, dMlsfieldGetHeight* pCallback,
				dReal width, dReal depth, int widthSamples, int depthSamples,
				dReal scale, dReal offset, dReal thickness, int bWrap );

ODE_API void dGeomMlsfieldDataBuildByte( dMlsfieldDataID d,
				const unsigned char* pHeightData, int bCopyHeightData,
				dReal width, dReal depth, int widthSamples, int depthSamples,
				dReal scale, dReal offset, dReal thickness,	int bWrap );

ODE_API void dGeomMlsfieldDataBuildShort( dMlsfieldDataID d,
				const short* pHeightData, int bCopyHeightData,
				dReal width, dReal depth, int widthSamples, int depthSamples,
				dReal scale, dReal offset, dReal thickness, int bWrap );

ODE_API void dGeomMlsfieldDataBuildSingle( dMlsfieldDataID d,
				const float* pHeightData, int bCopyHeightData,
				dReal width, dReal depth, int widthSamples, int depthSamples,
				dReal scale, dReal offset, dReal thickness, int bWrap );

ODE_API void dGeomMlsfieldDataBuildDouble( dMlsfieldDataID d,
				const double* pHeightData, int bCopyHeightData,
				dReal width, dReal depth, int widthSamples, int depthSamples,
				dReal scale, dReal offset, dReal thickness, int bWrap );

ODE_API void dGeomMlsfieldDataSetBounds( dMlsfieldDataID d,
				dReal minHeight, dReal maxHeight );

ODE_API dMlsfieldDataID dGeomMlsfieldGetMlsfieldData( dGeomID g );


class MlsFieldVertex;

//
// dxMlsfieldData
//
// Mlsfield Data structure
//

struct dxMlsfieldData
{
    dReal m_fWidth;				// World space heightfield dimension on X axis
    dReal m_fDepth;				// World space heightfield dimension on Z axis
    dReal m_fSampleWidth;		// Vertex spacing on X axis edge (== m_vWidth / (m_nWidthSamples-1))
    dReal m_fSampleDepth;		// Vertex spacing on Z axis edge (== m_vDepth / (m_nDepthSamples-1))
    dReal m_fSampleZXAspect;    // Relation of Z axis spacing to X axis spacing (== m_fSampleDepth / m_fSampleWidth)
    dReal m_fInvSampleWidth;		// Cache of inverse Vertex count on X axis edge (== m_vWidth / (m_nWidthSamples-1))
    dReal m_fInvSampleDepth;		// Cache of inverse Vertex count on Z axis edge (== m_vDepth / (m_nDepthSamples-1))

    dReal m_fHalfWidth;			// Cache of half of m_fWidth
    dReal m_fHalfDepth;			// Cache of half of m_fDepth

    dReal m_fMinHeight;        // Min sample height value (scaled and offset)
    dReal m_fMaxHeight;        // Max sample height value (scaled and offset)
    dReal m_fThickness;        // Surface thickness (added to bottom AABB)
    dReal m_fScale;            // Sample value multiplier
    dReal m_fOffset;           // Vertical sample offset

    int	m_nWidthSamples;       // Vertex count on X axis edge (number of samples)
    int	m_nDepthSamples;       // Vertex count on Z axis edge (number of samples)
    int m_bCopyHeightData;     // Do we own the sample data?
    int	m_bWrapMode;           // Heightfield wrapping mode (0=finite, 1=infinite)
    int m_nGetHeightMode;      // GetHeight mode ( 0=callback, 1=byte, 2=short, 3=float )

    const void* m_pHeightData; // Sample data array
    void* m_pUserData;         // Callback user data

    dContactGeom            m_contacts[MLSFIELDMAXCONTACTPERCELL];

    dMlsfieldGetHeight* m_pGetHeightCallback;		// Callback pointer.

    dxMlsfieldData();
    ~dxMlsfieldData();

    void SetData( int nWidthSamples, int nDepthSamples,
        dReal fWidth, dReal fDepth,
        dReal fScale, dReal fOffset,
        dReal fThickness, int bWrapMode );

    void ComputeHeightBounds();

    dReal GetHeight(int x, int z);
    dReal GetHeight(dReal x, dReal z);

};

class MlsFieldVertex
{
public:
    MlsFieldVertex(){};

    dVector3 vertex;
};

class MlsFieldRectangular
{
public:
    MlsFieldRectangular(){};

    MlsFieldVertex   *vertices[4];
};

//
// dxMlsfield
//
// Mlsfield geom structure
//

int dCollideMlsfield( dxGeom *o1, dxGeom *o2, int flags, dContactGeom* contact, int skip );

struct dxMlsfield : public dxGeom
{

    dxMlsfieldData* m_p_data;
    
    int dMlsfieldClass;

    dxMlsfield( dSpaceID space, dMlsfieldDataID data, int bPlaceable );
    ~dxMlsfield();

    void computeAABB();

    int dCollideMlsfieldZone( const int minX, const int maxX, const int minZ, const int maxZ,  
        dxGeom *o2, const int numMaxContacts,
        int flags, dContactGeom *contact, int skip );
    enum
    {
        TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_X = 4,
        TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_Z = 4,
        TEMP_RECTANGULAR_BUFFER_ELEMENT_COUNT_ALIGNMENT = 1 
    };

    static inline size_t AlignBufferSize(size_t value, size_t alignment) { dIASSERT((alignment & (alignment - 1)) == 0); return (value + (alignment - 1)) & ~(alignment - 1); }

    void  allocateRectangularBuffer(size_t numRect);   
    void  resetRectangularBuffer();    					
    void  allocateHeightBuffer(size_t numX, size_t numZ);
    void  resetHeightBuffer();
    MlsFieldRectangular *tempRectangularBuffer;       
    size_t				tempRectangularBufferSize; 

    MlsFieldVertex   **tempHeightBuffer;
    MlsFieldVertex   *tempHeightInstances;
    size_t              tempHeightBufferSizeX;
    size_t              tempHeightBufferSizeZ;

};


//------------------------------------------------------------------------------
#endif //_DMLSFIELD_H_
