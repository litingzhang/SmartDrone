/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include <cstdint>
#include <fstream>
#include <istream>
#include <ostream>
#include <string>

#include"DBoW2/FORB.h"
#include"DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM3
{

class ORBVocabulary : public DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
{
public:
  using Base = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;
  using Base::Base;

  bool loadFromBinaryFile(const std::string& filename)
  {
    std::ifstream in(filename, std::ios::binary);
    if(!in.is_open())
      return false;

    uint32_t magic = 0;
    uint32_t version = 0;
    if(!ReadPod(in, magic) || !ReadPod(in, version) || magic != 0x3142564Fu || version != 1u)
      return false;

    int weighting = 0;
    int scoring = 0;
    uint32_t nodeCount = 0;
    if(!ReadPod(in, this->m_k) || !ReadPod(in, this->m_L) ||
       !ReadPod(in, weighting) || !ReadPod(in, scoring) || !ReadPod(in, nodeCount))
      return false;

    this->m_weighting = static_cast<DBoW2::WeightingType>(weighting);
    this->m_scoring = static_cast<DBoW2::ScoringType>(scoring);
    this->m_nodes.clear();
    this->m_words.clear();
    this->m_nodes.resize(nodeCount);

    for(uint32_t i = 0; i < nodeCount; ++i)
    {
      auto& node = this->m_nodes[i];
      uint32_t childCount = 0;
      int rows = 0, cols = 0, type = 0;
      uint32_t dataBytes = 0;
      if(!ReadPod(in, node.id) || !ReadPod(in, node.weight) || !ReadPod(in, node.parent) ||
         !ReadPod(in, node.word_id) || !ReadPod(in, childCount) ||
         !ReadPod(in, rows) || !ReadPod(in, cols) || !ReadPod(in, type) || !ReadPod(in, dataBytes))
        return false;

      node.children.resize(childCount);
      if(childCount > 0 &&
         !in.read(reinterpret_cast<char*>(node.children.data()),
                  static_cast<std::streamsize>(childCount * sizeof(node.children[0]))))
        return false;

      node.descriptor.create(rows, cols, type);
      if(dataBytes > 0 &&
         !in.read(reinterpret_cast<char*>(node.descriptor.data), static_cast<std::streamsize>(dataBytes)))
        return false;
    }

    uint32_t wordCount = 0;
    if(!ReadPod(in, wordCount))
      return false;
    this->m_words.assign(wordCount, nullptr);
    for(uint32_t i = 0; i < nodeCount; ++i)
    {
      auto& node = this->m_nodes[i];
      if(node.word_id >= 0 && static_cast<size_t>(node.word_id) < this->m_words.size())
        this->m_words[node.word_id] = &node;
    }

    this->createScoringObject();
    return true;
  }

  bool saveToBinaryFile(const std::string& filename) const
  {
    std::ofstream out(filename, std::ios::binary | std::ios::trunc);
    if(!out.is_open())
      return false;

    const uint32_t magic = 0x3142564Fu;
    const uint32_t version = 1u;
    const int weighting = static_cast<int>(this->m_weighting);
    const int scoring = static_cast<int>(this->m_scoring);
    const uint32_t nodeCount = static_cast<uint32_t>(this->m_nodes.size());
    const uint32_t wordCount = static_cast<uint32_t>(this->m_words.size());
    if(!WritePod(out, magic) || !WritePod(out, version) ||
       !WritePod(out, this->m_k) || !WritePod(out, this->m_L) ||
       !WritePod(out, weighting) || !WritePod(out, scoring) || !WritePod(out, nodeCount))
      return false;

    for(const auto& node : this->m_nodes)
    {
      const uint32_t childCount = static_cast<uint32_t>(node.children.size());
      const int rows = node.descriptor.rows;
      const int cols = node.descriptor.cols;
      const int type = node.descriptor.type();
      const uint32_t dataBytes = static_cast<uint32_t>(node.descriptor.total() * node.descriptor.elemSize());
      if(!WritePod(out, node.id) || !WritePod(out, node.weight) || !WritePod(out, node.parent) ||
         !WritePod(out, node.word_id) || !WritePod(out, childCount) ||
         !WritePod(out, rows) || !WritePod(out, cols) || !WritePod(out, type) || !WritePod(out, dataBytes))
        return false;

      if(childCount > 0 &&
         !out.write(reinterpret_cast<const char*>(node.children.data()),
                    static_cast<std::streamsize>(childCount * sizeof(node.children[0]))))
        return false;
      if(dataBytes > 0 &&
         !out.write(reinterpret_cast<const char*>(node.descriptor.data), static_cast<std::streamsize>(dataBytes)))
        return false;
    }

    return WritePod(out, wordCount);
  }

private:
  template<typename T>
  static bool ReadPod(std::istream& in, T& value)
  {
    return static_cast<bool>(in.read(reinterpret_cast<char*>(&value), sizeof(T)));
  }

  template<typename T>
  static bool WritePod(std::ostream& out, const T& value)
  {
    return static_cast<bool>(out.write(reinterpret_cast<const char*>(&value), sizeof(T)));
  }
};

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
