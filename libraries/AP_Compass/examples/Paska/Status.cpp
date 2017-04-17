#include "Status.h"

//
//
//

bool pitotFailed()
{
  return !vpStatus.simulatorLink && pitotDevice.status();
}

bool alphaFailed()
{
  return !vpStatus.simulatorLink && alphaDevice.status();
}

