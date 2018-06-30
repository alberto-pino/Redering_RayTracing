#ifndef __TEXT_PARSER_H__
#define __TEXT_PARSER_H__

#include <gmtl/gmtl.h>
#include <transform.h>

class TextParser
{
private: ///////////////////////////////////////////////////////////////////////////////////////////////
	// DATA MEMBERS
	char*		m_pszBuffer;	//!< Buffer del texto
	int			m_iBufferSize;	//!< Tamaño del buffer
	char*		m_pszBegin;		//!< Puntero donde estamos leyendo

#ifdef _DEBUG
	string	m_sTextName;	//!< Nombre del texto. Cuando es un fichero, es el nombre del fichero
#endif // XDBG

public: ////////////////////////////////////////////////////////////////////////////////////////////////
	// LIFECYCLE
	TextParser() : m_pszBuffer(NULL), m_iBufferSize(0), m_pszBegin(NULL) {}
	TextParser(const string& sString) : m_pszBuffer(NULL), m_iBufferSize(0), m_pszBegin(NULL) { SetFromString(sString.c_str()); }
	TextParser(const char* szString) : m_pszBuffer(NULL), m_iBufferSize(0), m_pszBegin(NULL) { SetFromString(szString); }
	~TextParser() { Delete(); }

	/*! Libera la memoria.*/
	void Delete();

	/*! Inicializa a partir de un fichero. Lee el fichero completo en memoria.*/
	bool SetFromFile(const char* szFileName, bool bShowError=true);

	/*! Inicializa a partir de una cadena.
	 * @param pszName [in] "Nombre" de la cadena.*/
	void SetFromString(const char* pszString, const char* pszName=NULL);

	/*! Inicializa a partir de una cadena.
	 * @param pszName [in] "Nombre" de la cadena.*/
	void SetFromString(const string& sString, const char* pszName=NULL);

	/*! Inicializa a partir de un buffer.
	 * @param pBuffer [in] Buffer.
	 * @param iBufferSize [in] Tamaño del Buffer. El tamaño debe incluir el 0 final.
	 * @param pszName [in] "Nombre" del buffer.*/
	void SetFromBuffer(const char* pBuffer, int iBufferSize, const char* pszName=NULL);

	// OPERATORS

	// OPERATIONS
	/*! Funciones para coger datos desde el texto.*/
	bool GetToken(string& strToken, bool bConvertToUppercase=true);
	bool GetToken(char* szToken, unsigned int nMaxLength, bool bConvertToUppercase=true);
	/*! Devuelve el número de coma flotante que está a continuación.*/
	float GetNumber();
	/*! Devuelve el número entero que está a continuación.*/
	int GetIntNumber(bool* success = NULL);
	unsigned int GetUIntNumber() {return (unsigned int)GetIntNumber();}
	unsigned int GetHexNumber();
	/*! Devuelve el punto (3 números de coma flotante) que está a continuación.*/
	inline void GetPoint3(float& x, float& y, float& z);
	inline void GetVector3(gmtl::Vec3f& vVector) { vVector=GetVector3(); }
	inline gmtl::Vec3f GetVector3() { float x, y, z;  GetPoint3(x, y, z); return gmtl::Vec3f(x,y,z); }
	inline gmtl::Point3f GetPoint3() { float x, y, z;  GetPoint3(x, y, z); return gmtl::Point3f(x,y,z); }
    inline gmtl::Vec3f GetNormal() { float x, y, z; GetPoint3(x, y, z); return gmtl::Vec3f(x, y, z); }
	/*! Lee un punto 2D.*/
	inline void GetPoint2(float& x, float& y);
	inline gmtl::Point2f GetPoint2() {float x, y; GetPoint2(x, y); return gmtl::Point2f(x, y);}
	//inline void GetVector2(CU1Vect2& vVector2D) { GetPoint2(vVector2D.x, vVector2D.y); }
	/*! Lee un quaternion formado por su eje y el ángulo en grados.*/
	//inline void GetQuaternionFromAxisAngle(Quat& qQuaternion);
    inline void GetTransform(Transform& trans, bool column_major);

	/*! Obtiene el comentario que está a continuación. Debe ser un comentario tipo //.*/
	bool GetComment(char* szToken, int nMaxLength);

	/*! Obtiene el texto entre comillas.*/
	bool GetPhrase(char* szPhrase, unsigned int nMaxLength);
	bool GetPhrase(string& sPhrase, bool bToUpper=false);

	/*! Busca YES/SI/ON/1/TRUE y NO/OFF/0/FALSE.*/
	inline bool GetYesNo();
	//void GetYesNoFlag(DWORD &dwFlags, DWORD dwFlag) { if (GetYesNo()) dwFlags|=dwFlag; else dwFlags&=~dwFlag; }
	/*! Devuelve el siguiente carácter, sea el que sea (no se salta separadores).*/
	char GetNextCharacter();	
	/*! Devuelve la siguiente linea (sea lo que sea).*/
	bool GetLn(char* szLine, size_t nMaxLength)
	{
		string sLine;
		if( GetLn(sLine) )
		{
			int iSize=min(nMaxLength-1, sLine.size());
			memcpy(szLine, sLine.c_str(), iSize);
			szLine[iSize]=0;
			return true;
		}
		else
		{
			szLine[0]=0;
			return false;
		}
	}
	/*! Devuelve la siguiente linea (sea lo que sea).*/
	bool GetLn(string& sLine);
	bool GetNewLine(char* szLine, size_t nMaxLength) { return GetLn(szLine, nMaxLength); }
	/*! Deja en strToken, todos los caracteres hasta uno de los que haya en pszEnd, que devuelve.*/
	void GetUntilChar(string& strToken, const char *pszEnd, bool bJumpSeparators); 

	/*! Se salta el siguiente token que debe ser igual al indicado. Si no lo es, saca en el FXTrace un error
	 * grave y en DEBUG un assert
	 * @param pszToken [in] Token a saltarse
	 */
	void SkipToken(const char* pszToken);

	/*! Se salta la línea completa.
	 * @retval FALSE si no ha podido pasar a la siguiente, porque ya estábamos en la última.*/
	bool SkipLine();

	/*! Se salta el siguiente bloque. Un bloque está definido por las llaves { y }
	 * @param bFoundOpenBracket [in] debe ponerse a TRUE si ya hemos leído la primera llave de 
	 * apertura. Si es FALSE, se busca la apertura de llave.*/
	void SkipBlock(bool bFoundOpenBracket);

	/*! Salta hasta encontrar el token dado, posicionando el fichero en el siguiente caracter del token.
	 * Si no se encuentra, salta hasta el final del fichero.
	 * @param pszTokenToFind [in] Token a buscar.
	 * @retval TRUE si se ha encontrado el token.
	 * @retval FALSE si no se ha encontrado el token.*/
	bool SkipUntilToken(const char* pszTokenToFind);
	bool SkipUntilCharacter(char chChar);

	/*! Salta hasta encontrar un carácter que no esté en pszChars.*/
	void SkipCharacters(const char* pszChars, bool bJumpSeparators);

	/*! Obtiene en sBlock el bloque entre pszOpenBlock y pszCloseBlock (ambas no incluidas).
	 * Se usa, por ejemplo, para obtener los scripts de Lua, entre "[[" y "]]".*/
	void GetBlock(const char* pszOpenBlock, const char* pszCloseBlock, string& sBlock);

	/*! Rebobina el fichero los caracteres indicados.*/
	void Rewind(int nChars)
	{
		m_pszBegin-=nChars;
		assert(m_pszBegin>=m_pszBuffer);
	} // Rewind

	// ACCESS

	/*! @return Nombre del texto. Si es un fichero es el nombre del fichero. Si no, el que le hayamos puesto.*/
#ifdef _DEBUG
	const char*	GetTextName() const {return m_sTextName.c_str();}
	const char*	GetFileName() const {return m_sTextName.c_str();}
#endif // XDBG

	/*! @retval TRUE si se ha llegado al final del texto.*/
	bool Eof() const
	{
		return (m_pszBegin==NULL) || (m_pszBegin[0]==0) || (m_pszBegin>=&m_pszBuffer[m_iBufferSize-1]);
	} // Eof

	/* @return Posición desde el comienzo del texto donde está el puntero que lee */
	int GetPosition()
	{
		if( m_pszBegin==NULL )
			return m_iBufferSize-1;
		return (int)(m_pszBegin-m_pszBuffer);
	} // GetPosition

	/* Pone la posición del puntero de lectura donde se indica.
	 * @param iPosition [in] Posición desde el comienzo del texto donde se posiciona el puntero que lee */
	void SetPosition(int iPosition)
	{
		assert(iPosition<m_iBufferSize);
		m_pszBegin=&m_pszBuffer[iPosition];
	} // SetPosition

	/*! Devuelve TRUE si el siguiente caracter (excluyendo espacios, tabuladores y retornos
	 * de carros) es el indicado. Si es el caracter, se salta y se apunta al siguiente.
	 * Es util para determinar si lo que viene a continuación es un (, {, [ u otro 
	 * caracter que se salta al llamar a GetToken. */
	bool IsNextCharacter(char chCharacter);
	/*! Igual que IsNextCharacter(char szCharacter), pero podemos decirle que se salte los separadores y decidir
	 * si queremos que se salete el caracter cuando devuelva true.*/
	bool IsNextCharacter(char chCharacter, bool bSkipSeparators, bool bSkipCorrectCharacter);

	/*! Devuelve TRUE si el siguiente caracter es un número (se salta espacios, tabuladores y retornos de carro).*/
	bool IsNextNumber();
	/*! Devuelve TRUE si el siguiente "token" es un float (se salta espacios, tabuladores y retornos de carro).*/
	bool IsNextFloat();
	/*! Devuelve TRUE si el siguiente "token" es un signo (se salta espacios, tabuladores y retornos de carro).*/
	bool IsNextSign();

	/*! @retval TRUE si el siguiente caracter es un retorno de carro (saltándose espacios y tabuladores).
	 * Si es retorno de carro, se lo salta*/
	bool IsNextEndOfLine();

	/* Salta los separadores (espacios, tabuladores, comas, punto y coma, dos puntos, paréntesis, corchetes, retornos de carro...)
	 * @return Número de caracteres que se ha saltado.*/
	int JumpSeparators(bool bSkipOnlySpacesAndTabs=false) { return SkipSeparators(bSkipOnlySpacesAndTabs); }
	int SkipSeparators(bool bSkipOnlySpacesAndTabs=false);

};

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// Copyright (C) Crocodile Entertainment. All Rights Reserved ///////////////////////////
//
// PROJECT:		CrocoMathLib
// MODULE:		TextParser
// FILE NAME:   TextParser.inl
//
////////////////////////////////////////////////////////////////////////////////////////////////////////


//------------------------------------------------------------------------------------------------------
// Name: GetPhrase()
// Desc: Obtiene el texto entre comillas
// Date: 4-oct-07 Nacho Abril
//------------------------------------------------------------------------------------------------------
inline bool TextParser::GetPhrase(char* szPhrase, unsigned int nMaxLength)
{
    string sPhrase;
    if( GetPhrase(sPhrase) )
    {
        unsigned int iLength=min(static_cast<unsigned int>(sPhrase.size()), nMaxLength-1);
        memcpy(szPhrase, sPhrase.c_str(), iLength);
        szPhrase[iLength]=0;
        return true;
    }
    else
        return false;
} // GetPhrase

//------------------------------------------------------------------------------------------------------
// Name: GetPoint3()
// Desc: Obtiene un punto que son 3 floats
// Date: 13-nov-07 Nacho Abril
//------------------------------------------------------------------------------------------------------
inline void TextParser::GetPoint3(float& x, float& y, float& z)
{
    x=GetNumber();
    y=GetNumber();
    z=GetNumber();
} // GetPoint3

//------------------------------------------------------------------------------------------------------
// Name: GetPoint2()
// Desc: Obtiene un punto que son 2 floats
// Date: 25-nov-08 Nacho Abril
//------------------------------------------------------------------------------------------------------
inline void TextParser::GetPoint2(float& x, float& y)
{
    x=GetNumber();
    y=GetNumber();
} // GetPoint2

////------------------------------------------------------------------------------------------------------
//// Name: GetQuaternionFromAxisAngle()
//// Desc: Obtiene el quaternion a partir del eje (x, y, z) y ángulo en grados
//// Date: 13-nov-07 Nacho Abril
////------------------------------------------------------------------------------------------------------
//inline void TextParser::GetQuaternionFromAxisAngle(Quat& qQuaternion)
//{
//    float x, y, z;
//    Vector3 vAxis;
//    x=GetNumber();
//    y=GetNumber();
//    z=GetNumber();
//    float fAngleInDegrees=GetNumber();
//    vAxis=Vector3(x,y,z);
//    if( lengthSqr(vAxis)<FLT_EPSILON )
//        qQuaternion=Quat::identity();
//    else
//    {
//        qQuaternion=normalize(Quat::rotation(ToRadians(fAngleInDegrees), vAxis));
//    }
//} // GetQuaternionFromAxisAngle

inline void TextParser::GetTransform(Transform& trans, bool column_major)
{
    float cam_mat[4][4];
    if (column_major)
    {
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
            {
                cam_mat[row][column] = GetNumber();
            }
    }
    else
    {
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++row)
            {
                cam_mat[row][column] = GetNumber();
            }
    }
    
    trans = Transform(cam_mat);
}

/////////////////////////////////////////////////////////////////////////
// NAME:   CU1TextParser::GetYesNo
// CRDATE: 15/02/2008 16:15 Nacho Abril
/////////////////////////////////////////////////////////////////////////
inline bool TextParser::GetYesNo()
{
    ///---------------------------------
    /// Devuelve true si encuentra: /SI/ON/1/TRUE
    ///---------------------------------
    string sToken;
    GetToken(sToken);
    return (sToken=="YES") || (sToken=="SI") || (sToken=="1") || (sToken=="TRUE");
} // GetYesNo

unsigned int TextToUpper(string& sTxt);
unsigned int TextToLower(string& sTxt);
#endif