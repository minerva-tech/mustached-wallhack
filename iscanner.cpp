#include "pchbarrier.h"
#include "settings.h"
#include "defs.h"
#include "iscanner.h"
#include "scanner.h"

IScanner::IScanner(const Settings& settings) : 
//	m_impl(std::make_unique<Impl>(settings)) // I will migrate to normal compiler. Later.
	m_impl(new Impl(settings))
{ 
}

IScanner::~IScanner()
{
}

void IScanner::process() 
{ 
	m_impl->process(); 
}
