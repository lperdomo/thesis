#include "database.h"

Database::Database()
{

}

void Database::load()
{
    filesRGB.clear();
    filesRGB.push_back("fr1room/database/1305031910.835208.png");
    filesRGB.push_back("fr1room/database/1305031911.997252.png");
    filesRGB.push_back("fr1room/database/1305031915.264979.png");
    filesRGB.push_back("fr1room/database/1305031916.665139.png");
    filesRGB.push_back("fr1room/database/1305031919.001053.png");
    filesRGB.push_back("fr1room/database/1305031920.865163.png");
    filesRGB.push_back("fr1room/database/1305031922.265183.png");
    filesRGB.push_back("fr1room/database/1305031924.601161.png");
    filesRGB.push_back("fr1room/database/1305031926.701042.png");
    filesRGB.push_back("fr1room/database/1305031927.869143.png");
    filesRGB.push_back("fr1room/database/1305031929.033640.png");
    filesRGB.push_back("fr1room/database/1305031929.969176.png");
    filesRGB.push_back("fr1room/database/1305031931.133042.png");
    filesRGB.push_back("fr1room/database/1305031934.400928.png");
    filesRGB.push_back("fr1room/database/1305031938.136885.png");
    filesRGB.push_back("fr1room/database/1305031939.536755.png");
    filesDepth.clear();
    filesDepth.push_back("fr1room/database/1305031910.835215.png");
    filesDepth.push_back("fr1room/database/1305031912.004430.png");
    filesDepth.push_back("fr1room/database/1305031915.276431.png");
    filesDepth.push_back("fr1room/database/1305031916.674514.png");
    filesDepth.push_back("fr1room/database/1305031919.013079.png");
    filesDepth.push_back("fr1room/database/1305031920.879246.png");
    filesDepth.push_back("fr1room/database/1305031922.284278.png");
    filesDepth.push_back("fr1room/database/1305031924.620515.png");
    filesDepth.push_back("fr1room/database/1305031926.716774.png");
    filesDepth.push_back("fr1room/database/1305031927.887591.png");
    filesDepth.push_back("fr1room/database/1305031929.052047.png");
    filesDepth.push_back("fr1room/database/1305031929.992281.png");
    filesDepth.push_back("fr1room/database/1305031931.156194.png");
    filesDepth.push_back("fr1room/database/1305031934.427107.png");
    filesDepth.push_back("fr1room/database/1305031938.168307.png");
    filesDepth.push_back("fr1room/database/1305031939.567564.png");
}

void Database::loadQueries()
{
    filesRGB.clear();
    filesRGB.push_back("fr1room/query/1305031910.765238.png");
    filesRGB.push_back("fr1room/query/1305031910.797230.png");
    filesRGB.push_back("fr1room/query/1305031910.865025.png");
    filesRGB.push_back("fr1room/query/1305031910.897222.png");
    filesRGB.push_back("fr1room/query/1305031911.933063.png");
    filesRGB.push_back("fr1room/query/1305031911.965140.png");
    filesRGB.push_back("fr1room/query/1305031912.033135.png");
    filesRGB.push_back("fr1room/query/1305031912.065365.png");
    filesRGB.push_back("fr1room/query/1305031915.197108.png");
    filesRGB.push_back("fr1room/query/1305031915.233111.png");
    filesRGB.push_back("fr1room/query/1305031915.297152.png");
    filesRGB.push_back("fr1room/query/1305031915.333207.png");
    filesRGB.push_back("fr1room/query/1305031916.601238.png");
    filesRGB.push_back("fr1room/query/1305031916.633156.png");
    filesRGB.push_back("fr1room/query/1305031916.701048.png");
    filesRGB.push_back("fr1room/query/1305031916.733056.png");
    filesRGB.push_back("fr1room/query/1305031918.933071.png");
    filesRGB.push_back("fr1room/query/1305031918.965058.png");
    filesRGB.push_back("fr1room/query/1305031919.033082.png");
    filesRGB.push_back("fr1room/query/1305031919.065147.png");
    filesRGB.push_back("fr1room/query/1305031920.801031.png");
    filesRGB.push_back("fr1room/query/1305031920.833132.png");
    filesRGB.push_back("fr1room/query/1305031920.901037.png");
    filesRGB.push_back("fr1room/query/1305031920.933077.png");
    filesRGB.push_back("fr1room/query/1305031922.201057.png");
    filesRGB.push_back("fr1room/query/1305031922.233248.png");
    filesRGB.push_back("fr1room/query/1305031922.301137.png");
    filesRGB.push_back("fr1room/query/1305031922.333215.png");
    filesRGB.push_back("fr1room/query/1305031924.533245.png");
    filesRGB.push_back("fr1room/query/1305031924.565264.png");
    filesRGB.push_back("fr1room/query/1305031924.633221.png");
    filesRGB.push_back("fr1room/query/1305031924.665187.png");
    filesRGB.push_back("fr1room/query/1305031926.633127.png");
    filesRGB.push_back("fr1room/query/1305031926.669140.png");
    filesRGB.push_back("fr1room/query/1305031926.733207.png");
    filesRGB.push_back("fr1room/query/1305031926.769141.png");
    filesRGB.push_back("fr1room/query/1305031927.801109.png");
    filesRGB.push_back("fr1room/query/1305031927.833282.png");
    filesRGB.push_back("fr1room/query/1305031927.901130.png");
    filesRGB.push_back("fr1room/query/1305031927.933465.png");
    filesRGB.push_back("fr1room/query/1305031928.969097.png");
    filesRGB.push_back("fr1room/query/1305031929.001155.png");
    filesRGB.push_back("fr1room/query/1305031929.069089.png");
    filesRGB.push_back("fr1room/query/1305031929.101213.png");
    filesRGB.push_back("fr1room/query/1305031929.901292.png");
    filesRGB.push_back("fr1room/query/1305031929.933176.png");
    filesRGB.push_back("fr1room/query/1305031930.001350.png");
    filesRGB.push_back("fr1room/query/1305031930.033650.png");
    filesRGB.push_back("fr1room/query/1305031931.068905.png");
    filesRGB.push_back("fr1room/query/1305031931.100819.png");
    filesRGB.push_back("fr1room/query/1305031931.168900.png");
    filesRGB.push_back("fr1room/query/1305031931.200696.png");
    filesRGB.push_back("fr1room/query/1305031934.336871.png");
    filesRGB.push_back("fr1room/query/1305031934.368906.png");
    filesRGB.push_back("fr1room/query/1305031934.436865.png");
    filesRGB.push_back("fr1room/query/1305031934.469287.png");
    filesRGB.push_back("fr1room/query/1305031938.068761.png");
    filesRGB.push_back("fr1room/query/1305031938.100744.png");
    filesRGB.push_back("fr1room/query/1305031938.168914.png");
    filesRGB.push_back("fr1room/query/1305031938.200883.png");
    filesRGB.push_back("fr1room/query/1305031939.468851.png");
    filesRGB.push_back("fr1room/query/1305031939.500839.png");
    filesRGB.push_back("fr1room/query/1305031939.568772.png");
    filesRGB.push_back("fr1room/query/1305031939.600838.png");
    filesDepth.clear();
    filesDepth.push_back("fr1room/query/1305031910.771502.png");
    filesDepth.push_back("fr1room/query/1305031910.803249.png");
    filesDepth.push_back("fr1room/query/1305031910.871167.png");
    filesDepth.push_back("fr1room/query/1305031910.903682.png");
    filesDepth.push_back("fr1room/query/1305031911.940338.png");
    filesDepth.push_back("fr1room/query/1305031911.972044.png");
    filesDepth.push_back("fr1room/query/1305031912.040573.png");
    filesDepth.push_back("fr1room/query/1305031912.069771.png");
    filesDepth.push_back("fr1room/query/1305031915.209278.png");
    filesDepth.push_back("fr1room/query/1305031915.244223.png");
    filesDepth.push_back("fr1room/query/1305031915.306226.png");
    filesDepth.push_back("fr1room/query/1305031915.341488.png");
    filesDepth.push_back("fr1room/query/1305031916.610788.png");
    filesDepth.push_back("fr1room/query/1305031916.643191.png");
    filesDepth.push_back("fr1room/query/1305031916.711186.png");
    filesDepth.push_back("fr1room/query/1305031916.742388.png");
    filesDepth.push_back("fr1room/query/1305031918.948107.png");
    filesDepth.push_back("fr1room/query/1305031918.980882.png");
    filesDepth.push_back("fr1room/query/1305031919.047888.png");
    filesDepth.push_back("fr1room/query/1305031919.081202.png");
    filesDepth.push_back("fr1room/query/1305031920.813643.png");
    filesDepth.push_back("fr1room/query/1305031920.848339.png");
    filesDepth.push_back("fr1room/query/1305031920.915238.png");
    filesDepth.push_back("fr1room/query/1305031920.947777.png");
    filesDepth.push_back("fr1room/query/1305031922.217879.png");
    filesDepth.push_back("fr1room/query/1305031922.248353.png");
    filesDepth.push_back("fr1room/query/1305031922.315988.png");
    filesDepth.push_back("fr1room/query/1305031922.348617.png");
    filesDepth.push_back("fr1room/query/1305031924.552273.png");
    filesDepth.push_back("fr1room/query/1305031924.584109.png");
    filesDepth.push_back("fr1room/query/1305031924.652060.png");
    filesDepth.push_back("fr1room/query/1305031924.684319.png");
    filesDepth.push_back("fr1room/query/1305031926.655592.png");
    filesDepth.push_back("fr1room/query/1305031926.686940.png");
    filesDepth.push_back("fr1room/query/1305031926.754636.png");
    filesDepth.push_back("fr1room/query/1305031926.787267.png");
    filesDepth.push_back("fr1room/query/1305031927.825135.png");
    filesDepth.push_back("fr1room/query/1305031927.855551.png");
    filesDepth.push_back("fr1room/query/1305031927.923620.png");
    filesDepth.push_back("fr1room/query/1305031927.955459.png");
    filesDepth.push_back("fr1room/query/1305031928.991477.png");
    filesDepth.push_back("fr1room/query/1305031929.023472.png");
    filesDepth.push_back("fr1room/query/1305031929.091469.png");
    filesDepth.push_back("fr1room/query/1305031929.123397.png");
    filesDepth.push_back("fr1room/query/1305031929.920589.png");
    filesDepth.push_back("fr1room/query/1305031929.960336.png");
    filesDepth.push_back("fr1room/query/1305031930.024129.png");
    filesDepth.push_back("fr1room/query/1305031930.059766.png");
    filesDepth.push_back("fr1room/query/1305031931.091660.png");
    filesDepth.push_back("fr1room/query/1305031931.126327.png");
    filesDepth.push_back("fr1room/query/1305031931.187879.png");
    filesDepth.push_back("fr1room/query/1305031931.226834.png");
    filesDepth.push_back("fr1room/query/1305031934.363432.png");
    filesDepth.push_back("fr1room/query/1305031934.395278.png");
    filesDepth.push_back("fr1room/query/1305031934.460046.png");
    filesDepth.push_back("fr1room/query/1305031934.495228.png");
    filesDepth.push_back("fr1room/query/1305031938.100010.png");
    filesDepth.push_back("fr1room/query/1305031938.131856.png");
    filesDepth.push_back("fr1room/query/1305031938.199974.png");
    filesDepth.push_back("fr1room/query/1305031938.231649.png");
    filesDepth.push_back("fr1room/query/1305031939.499844.png");
    filesDepth.push_back("fr1room/query/1305031939.535364.png");
    filesDepth.push_back("fr1room/query/1305031939.600160.png");
    filesDepth.push_back("fr1room/query/1305031939.635762.png");

    queryTarget.clear();
    queryTarget.push_back(0);
    queryTarget.push_back(0);
    queryTarget.push_back(0);
    queryTarget.push_back(0);
    queryTarget.push_back(1);
    queryTarget.push_back(1);
    queryTarget.push_back(1);
    queryTarget.push_back(1);
    queryTarget.push_back(2);
    queryTarget.push_back(2);
    queryTarget.push_back(2);
    queryTarget.push_back(2);
    queryTarget.push_back(3);
    queryTarget.push_back(3);
    queryTarget.push_back(3);
    queryTarget.push_back(3);
    queryTarget.push_back(4);
    queryTarget.push_back(4);
    queryTarget.push_back(4);
    queryTarget.push_back(4);
    queryTarget.push_back(5);
    queryTarget.push_back(5);
    queryTarget.push_back(5);
    queryTarget.push_back(5);
    queryTarget.push_back(6);
    queryTarget.push_back(6);
    queryTarget.push_back(6);
    queryTarget.push_back(6);
    queryTarget.push_back(7);
    queryTarget.push_back(7);
    queryTarget.push_back(7);
    queryTarget.push_back(7);
    queryTarget.push_back(8);
    queryTarget.push_back(8);
    queryTarget.push_back(8);
    queryTarget.push_back(8);
    queryTarget.push_back(9);
    queryTarget.push_back(9);
    queryTarget.push_back(9);
    queryTarget.push_back(9);
    queryTarget.push_back(10);
    queryTarget.push_back(10);
    queryTarget.push_back(10);
    queryTarget.push_back(10);
    queryTarget.push_back(11);
    queryTarget.push_back(11);
    queryTarget.push_back(11);
    queryTarget.push_back(11);
    queryTarget.push_back(12);
    queryTarget.push_back(12);
    queryTarget.push_back(12);
    queryTarget.push_back(12);
    queryTarget.push_back(13);
    queryTarget.push_back(13);
    queryTarget.push_back(13);
    queryTarget.push_back(13);
    queryTarget.push_back(14);
    queryTarget.push_back(14);
    queryTarget.push_back(14);
    queryTarget.push_back(14);
    queryTarget.push_back(15);
    queryTarget.push_back(15);
    queryTarget.push_back(15);
    queryTarget.push_back(15);
}

void Database::loadTest1()
{
    filesRGB.clear();
    filesRGB.push_back("fr1room/test/1305031910.797230.png");
    filesDepth.clear();
    filesDepth.push_back("fr1room/test/1305031910.803249.png");
}

void Database::loadTest2()
{
    filesRGB.clear();
    filesRGB.push_back("fr1room/test/1305031910.797230.png");
    filesRGB.push_back("fr1room/test/1305031910.835208.png");
    filesRGB.push_back("fr1room/test/1305031911.765284.png");
    filesRGB.push_back("fr1room/test/1305031913.933123.png");
    filesRGB.push_back("fr1room/test/1305031910.897222.png");
    filesDepth.clear();
    filesDepth.push_back("fr1room/test/1305031910.803249.png");
    filesDepth.push_back("fr1room/test/1305031910.835215.png");
    filesDepth.push_back("fr1room/test/1305031911.771372.png");
    filesDepth.push_back("fr1room/test/1305031913.939898.png");
    filesDepth.push_back("fr1room/test/1305031910.903682.png");
}

void Database::loadTest2Queries()
{
    filesRGB.clear();
    filesRGB.push_back("fr1room/test/1305031910.797230.png");
    filesRGB.push_back("fr1room/test/1305031910.835208.png");
    filesRGB.push_back("fr1room/test/1305031911.765284.png");
    filesRGB.push_back("fr1room/test/1305031913.933123.png");
    filesRGB.push_back("fr1room/test/1305031910.897222.png");
    filesDepth.clear();
    filesDepth.push_back("fr1room/test/1305031910.803249.png");
    filesDepth.push_back("fr1room/test/1305031910.835215.png");
    filesDepth.push_back("fr1room/test/1305031911.771372.png");
    filesDepth.push_back("fr1room/test/1305031913.939898.png");
    filesDepth.push_back("fr1room/test/1305031910.903682.png");
    queryTarget.clear();
    queryTarget.push_back(0);
    queryTarget.push_back(1);
    queryTarget.push_back(2);
    queryTarget.push_back(3);
    queryTarget.push_back(4);
}


void Database::loadTest3()
{
    filesRGB.clear();
    filesRGB.push_back("fr1room/query/1305031910.965249.png");
    filesDepth.clear();
    filesDepth.push_back("fr1room/query/1305031910.971338.png");
    queryTarget.clear();
    queryTarget.push_back(0);
}
