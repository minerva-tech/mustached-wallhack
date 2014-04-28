#ifndef ISCANNER_H
#define ISCANNER_H

class IScanner {
public:
	IScanner(const Settings& settings);
	~IScanner();

	void process();

private:
	class Impl;

	std::unique_ptr<Impl> m_impl;
};


#endif
