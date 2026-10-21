#include "RLS.h"
#include "FreeRTOS.h"

#define RLS_ALLOCATION_TAG 0x524C5346UL

static void RLS_FreeData(Recursive_Least_Squares_Info_TypeDef *RLS)
{
    vPortFree(RLS->Data.W);
    vPortFree(RLS->Data.X);
    vPortFree(RLS->Data.XT);
    vPortFree(RLS->Data.Lamda);
    vPortFree(RLS->Data.P);
    vPortFree(RLS->Data.Y);
    vPortFree(RLS->Data.U);
    vPortFree(RLS->Data.E);
    vPortFree(RLS->Data.K_Numerator);
    vPortFree(RLS->Data.K_Denominator);
    vPortFree(RLS->Data.K);
    vPortFree(RLS->Data.Cache_Matrix[0]);
    vPortFree(RLS->Data.Cache_Matrix[1]);
    vPortFree(RLS->Data.Cache_Vector[0]);
    vPortFree(RLS->Data.Cache_Vector[1]);
    vPortFree(RLS->Data.Output);
}

void RLS_Deinit(Recursive_Least_Squares_Info_TypeDef *RLS)
{
    if (RLS == NULL) {
        return;
    }
    if (RLS->AllocationTag == RLS_ALLOCATION_TAG) {
        RLS_FreeData(RLS);
    }
    memset(RLS, 0, sizeof(*RLS));
}

void RLS_Init(Recursive_Least_Squares_Info_TypeDef *RLS,uint8_t X_Size,uint8_t P_Size,uint8_t Y_Size){
	if (RLS == NULL || X_Size == 0U || P_Size == 0U || Y_Size == 0U) {
		return;
	}
	if (RLS->AllocationTag == RLS_ALLOCATION_TAG) {
		RLS_Deinit(RLS);
	} else {
		memset(RLS, 0, sizeof(*RLS));
	}


	 RLS->sizeof_float = sizeof(float);
	
   RLS->X_Size = X_Size;
	 RLS->P_Size = P_Size;
   RLS->Y_Size = Y_Size;

#define RLS_ALLOCATE(member, bytes) do { \
        (member) = (float *)pvPortMalloc(bytes); \
        if ((member) == NULL) { \
            RLS->MatStatus = ARM_MATH_ARGUMENT_ERROR; \
            goto allocation_failed; \
        } \
        memset((member), 0, (bytes)); \
    } while (0)
   
	 RLS_ALLOCATE(RLS->Data.W, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.W, X_Size, 1, (float *)RLS->Data.W);
	
	 RLS_ALLOCATE(RLS->Data.X, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.X, X_Size, 1, (float *)RLS->Data.X);
	
	 RLS_ALLOCATE(RLS->Data.XT, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.XT,1, X_Size, (float *)RLS->Data.XT);
	
	 RLS_ALLOCATE(RLS->Data.Lamda, RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.Lamda,1, 1, (float *)RLS->Data.Lamda);
	
	 RLS_ALLOCATE(RLS->Data.P, RLS->sizeof_float * P_Size * P_Size);
   Matrix_Init(&RLS->Mat.P, P_Size, P_Size, (float *)RLS->Data.P);
    	
	 RLS_ALLOCATE(RLS->Data.Y, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.Y, Y_Size, 1, (float *)RLS->Data.Y);      

	 RLS_ALLOCATE(RLS->Data.U, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.U, Y_Size, 1, (float *)RLS->Data.U);   

	 RLS_ALLOCATE(RLS->Data.E, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.E, Y_Size, 1, (float *)RLS->Data.E);     
	 
	 RLS_ALLOCATE(RLS->Data.K_Numerator, RLS->sizeof_float * P_Size * P_Size);
   Matrix_Init(&RLS->Mat.K_Numerator, P_Size, P_Size, (float *)RLS->Data.K_Numerator);
	 
	 RLS_ALLOCATE(RLS->Data.K_Denominator, RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.K_Denominator, 1, 1, (float *)RLS->Data.K_Denominator);
	 
	 RLS_ALLOCATE(RLS->Data.K, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.K, X_Size, 1, (float *)RLS->Data.K);
	 
	 RLS_ALLOCATE(RLS->Data.Cache_Matrix[0], RLS->sizeof_float * X_Size * X_Size);
   Matrix_Init(&RLS->Mat.Cache_Matrix[0], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[0]);

	 RLS_ALLOCATE(RLS->Data.Cache_Matrix[1], RLS->sizeof_float * X_Size * X_Size);
   Matrix_Init(&RLS->Mat.Cache_Matrix[1], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[1]);
	 
	 RLS_ALLOCATE(RLS->Data.Output, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.Output, RLS->X_Size, 1, (float *)RLS->Data.Output);
	RLS->Initialized = true;
	RLS->AllocationTag = RLS_ALLOCATION_TAG;
	RLS->MatStatus = ARM_MATH_SUCCESS;
#undef RLS_ALLOCATE
	return;

allocation_failed:
#undef RLS_ALLOCATE
	RLS_FreeData(RLS);
	memset(RLS, 0, sizeof(*RLS));
	RLS->MatStatus = ARM_MATH_ARGUMENT_ERROR;
}

void RLS_Update(Recursive_Least_Squares_Info_TypeDef *RLS){
	if (RLS == NULL || !RLS->Initialized) {
		return;
	}
 
    
  //E(n) = Y(n) - U(n)
	RLS->MatStatus = Matrix_Subtract(&RLS->Mat.Y,&RLS->Mat.U,&RLS->Mat.E);

  //P(n-1)*X(n) 
	RLS->Mat.K_Numerator.numRows = RLS->Mat.P.numRows;
  RLS->Mat.K_Numerator.numCols = RLS->Mat.X.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.P,& RLS->Mat.X,&RLS->Mat.K_Numerator);
	
	//XT(n) 
  RLS->MatStatus = Matrix_Transpose(&RLS->Mat.X,&RLS->Mat.XT);	
	
	//XT(n)*P(n-1)
	RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.XT.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.XT,& RLS->Mat.P,&RLS->Mat.Cache_Matrix[0]);
	
	
	//XT(n)*P(n-1)*X(n)
  RLS->Mat.Cache_Matrix[1].numRows = 1;
  RLS->Mat.Cache_Matrix[1].numCols = 1;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0],& RLS->Mat.X,&RLS->Mat.Cache_Matrix[1]);
	
	//lamda + XT(n)*P(n-1)*X(n)
	RLS->Mat.Cache_Matrix[0].numRows = 1;
  RLS->Mat.Cache_Matrix[0].numCols = 1;
	RLS->MatStatus = Matrix_Add(&RLS->Mat.Lamda,&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0]);

  //1/(lamda + XT(n)*P(n-1)*X(n))
  RLS->MatStatus = Matrix_Inverse(&RLS->Mat.Cache_Matrix[0],&RLS->Mat.K_Denominator);


 //K = P(n-1)*X(n) / lamda + XT(n)*P(n-1)*X(n)
	RLS->Mat.K.numRows = RLS->Mat.K_Numerator.numRows;
  RLS->Mat.K.numCols = RLS->Mat.K_Denominator.numCols;;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K_Numerator,& RLS->Mat.K_Denominator,&RLS->Mat.K);
 

// K * XT(n) 	
  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.XT.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K,& RLS->Mat.XT,&RLS->Mat.Cache_Matrix[0]);

// K * XT(n) * P(n-1)
  RLS->Mat.Cache_Matrix[1].numRows = RLS->Mat.Cache_Matrix[0].numRows;
  RLS->Mat.Cache_Matrix[1].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0],&RLS->Mat.P,&RLS->Mat.Cache_Matrix[1]);

// P(n-1) -  K * XT(n) * P(n-1)

  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.P.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Subtract(&RLS->Mat.P,&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0]);

// 1/lamda
  RLS->Mat.Cache_Matrix[1].numRows = 1;
  RLS->Mat.Cache_Matrix[1].numCols = 1;
	RLS->Data.Cache_Matrix[1][0] = 1/RLS->Data.Lamda[0];

//P(n) = 1/lamda *  (P(n-1) -  K * XT(n) * P(n-1))
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0],&RLS->Mat.P);

//W(n) = W(n-1) + K*E(n)

  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.E.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K,&RLS->Mat.E,&RLS->Mat.Cache_Matrix[0]);

  RLS->MatStatus = Matrix_Add(&RLS->Mat.W,&RLS->Mat.Cache_Matrix[0],&RLS->Mat.Output);

  RLS->Data.W[0] = RLS->Data.Output[0];
  if (RLS->X_Size > 1U) {
    RLS->Data.W[1] = RLS->Data.Output[1];
  }


}
	






