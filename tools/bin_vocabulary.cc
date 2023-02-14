#include "ORBVocabulary.h"
using namespace std;

bool load_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
    cout << "Load vocabulary From text" << endl;
    bool res = voc->loadFromTextFile(infile);
    return res;
}

void load_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
    cout << "Load vocabulary From xml" << endl;
    voc->load(infile);
}

void load_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
    cout << "Load vocabulary From binary" << endl;
    voc->loadFromBinaryFile(infile);
}

void save_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
    cout << "Save vocabulary as xml" << endl;
    voc->save(outfile);
}

void save_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
    cout << "Save vocabulary as text" << endl;
    voc->saveToTextFile(outfile);
}

void save_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
    cout << "Save vocabulary as binary" << endl;
    voc->saveToBinaryFile(outfile);
}


int main(int argc, char **argv) {
    ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();

    load_as_text(voc, "Vocabulary/ORBvoc.txt");
    // save_as_binary(voc, "Vocabulary/ORBvoc.bin");
    load_as_binary(voc, "Vocabulary/ORBvoc.bin");
    return 0;
}