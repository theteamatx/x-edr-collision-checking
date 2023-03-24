#include "collision_checking/assembly/assembly_connectivity.h"

#include <string>
#include <vector>

#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly/proto_utils.h"
#include "collision_checking/test_utils.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {
using ::testing::ElementsAre;
using ::testing::IsEmpty;
using testing::ParseTextProtoOrDie;
using ::testing::UnorderedElementsAre;
using ::testing::UnorderedElementsAreArray;

constexpr absl::string_view kTestAssembly = R"pb(
  name: "test"
  links: { name: "root" }
  links: { name: "A" }
  links: { name: "B" }
  links: { name: "C" }
  links: { name: "D" }
  links: { name: "E" }

  links: { name: "F" }
  links: { name: "G" }
  links: { name: "H" }

  joints: {
    name: "root_A"
    parent_link_name: "root"
    child_link_name: "A"
    type: FIXED
  }
  joints: {
    name: "root_F"
    parent_link_name: "root"
    child_link_name: "F"
    type: REVOLUTE
    axis: { vec: [ 1, 0, 0 ] }
  }
  joints: {
    name: "F_H"
    parent_link_name: "F"
    child_link_name: "H"
    type: REVOLUTE
    axis: { vec: [ 1, 0, 0 ] }
  }
  joints: {
    name: "F_G"
    parent_link_name: "F"
    child_link_name: "G"
    type: REVOLUTE
    axis: { vec: [ 1, 0, 0 ] }
  }
  joints: { name: "A_B" parent_link_name: "A" child_link_name: "B" type: FIXED }
  joints: { name: "A_C" parent_link_name: "A" child_link_name: "C" type: FIXED }
  joints: {
    name: "C_D"
    parent_link_name: "C"
    child_link_name: "D"
    type: REVOLUTE
    axis: { vec: [ 1, 0, 0 ] }
  }
  joints: {
    name: "B_E"
    parent_link_name: "B"
    child_link_name: "E"
    type: REVOLUTE
    axis: { vec: [ 1, 0, 0 ] }
  }
)pb";

TEST(AssemblyConnectivityTest, FindsConnectedLinksForOnlyFixedLinks) {
  CC_ASSERT_OK_AND_ASSIGN(
      const Assembly assembly,
      FromProto(ParseTextProtoOrDie<AssemblyProto>(kTestAssembly)));

  AssemblyConnectivity connectivity(assembly, /*max_movable_joints=*/0);

  EXPECT_THAT(connectivity.GetConnectedLinkIndices(-1), IsEmpty());
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(assembly.GetLinkCount() + 5),
              IsEmpty());

  const int root_index =
      ABSL_DIE_IF_NULL(assembly.FindLink("root"))->GetIndex();
  const int a_index = ABSL_DIE_IF_NULL(assembly.FindLink("A"))->GetIndex();
  const int b_index = ABSL_DIE_IF_NULL(assembly.FindLink("B"))->GetIndex();
  const int c_index = ABSL_DIE_IF_NULL(assembly.FindLink("C"))->GetIndex();
  const int d_index = ABSL_DIE_IF_NULL(assembly.FindLink("D"))->GetIndex();
  const int e_index = ABSL_DIE_IF_NULL(assembly.FindLink("E"))->GetIndex();
  const int f_index = ABSL_DIE_IF_NULL(assembly.FindLink("F"))->GetIndex();
  const int g_index = ABSL_DIE_IF_NULL(assembly.FindLink("G"))->GetIndex();
  const int h_index = ABSL_DIE_IF_NULL(assembly.FindLink("H"))->GetIndex();

  EXPECT_THAT(connectivity.GetConnectedLinkIndices(root_index),
              UnorderedElementsAre(root_index, a_index, b_index, c_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(a_index),
              UnorderedElementsAre(a_index, root_index, c_index, b_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(b_index),
              UnorderedElementsAre(b_index, root_index, a_index, c_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(c_index),
              UnorderedElementsAre(c_index, root_index, a_index, b_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(d_index),
              ElementsAre(d_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(e_index),
              ElementsAre(e_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(f_index),
              ElementsAre(f_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(g_index),
              ElementsAre(g_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(h_index),
              ElementsAre(h_index));
}

TEST(AssemblyConnectivityTest, FindsConnectedLinksForOneMovableJoint) {
  CC_ASSERT_OK_AND_ASSIGN(
      const Assembly assembly,
      FromProto(ParseTextProtoOrDie<AssemblyProto>(kTestAssembly)));

  AssemblyConnectivity connectivity(assembly, /*max_movable_joints=*/1);

  EXPECT_THAT(connectivity.GetConnectedLinkIndices(-1), IsEmpty());
  const int root_index =
      ABSL_DIE_IF_NULL(assembly.FindLink("root"))->GetIndex();
  const int a_index = ABSL_DIE_IF_NULL(assembly.FindLink("A"))->GetIndex();
  const int b_index = ABSL_DIE_IF_NULL(assembly.FindLink("B"))->GetIndex();
  const int c_index = ABSL_DIE_IF_NULL(assembly.FindLink("C"))->GetIndex();
  const int d_index = ABSL_DIE_IF_NULL(assembly.FindLink("D"))->GetIndex();
  const int e_index = ABSL_DIE_IF_NULL(assembly.FindLink("E"))->GetIndex();
  const int f_index = ABSL_DIE_IF_NULL(assembly.FindLink("F"))->GetIndex();
  const int g_index = ABSL_DIE_IF_NULL(assembly.FindLink("G"))->GetIndex();
  const int h_index = ABSL_DIE_IF_NULL(assembly.FindLink("H"))->GetIndex();

  EXPECT_THAT(connectivity.GetConnectedLinkIndices(root_index),
              UnorderedElementsAre(root_index, a_index, b_index, c_index,
                                   f_index, e_index, d_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(a_index),
              UnorderedElementsAre(a_index, root_index, c_index, b_index,
                                   f_index, e_index, d_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(b_index),
              UnorderedElementsAre(b_index, root_index, a_index, c_index,
                                   f_index, e_index, d_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(c_index),
              UnorderedElementsAre(c_index, root_index, a_index, b_index,
                                   f_index, e_index, d_index));
  EXPECT_THAT(
      connectivity.GetConnectedLinkIndices(d_index),
      UnorderedElementsAre(d_index, c_index, b_index, a_index, root_index));
  EXPECT_THAT(
      connectivity.GetConnectedLinkIndices(e_index),
      UnorderedElementsAre(e_index, b_index, c_index, a_index, root_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(f_index),
              UnorderedElementsAre(f_index, g_index, h_index, root_index,
                                   a_index, b_index, c_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(g_index),
              UnorderedElementsAre(g_index, f_index));
  EXPECT_THAT(connectivity.GetConnectedLinkIndices(h_index),
              UnorderedElementsAre(h_index, f_index));
}

TEST(AssemblyConnectivityTest, FindsAllIfMaxMovableJointsLargeEnough) {
  CC_ASSERT_OK_AND_ASSIGN(
      const Assembly assembly,
      FromProto(ParseTextProtoOrDie<AssemblyProto>(kTestAssembly)));

  std::vector<int> all_link_indices;
  for (const auto& link : assembly.GetLinks()) {
    all_link_indices.push_back(link.GetIndex());
  }
  AssemblyConnectivity connectivity(
      assembly, /*max_movable_joints=*/all_link_indices.size() - 1);

  for (const auto& index : all_link_indices) {
    SCOPED_TRACE(absl::StrCat(
        "index: ", index, " (name= ", assembly.GetLink(index).GetName(), ")"));
    EXPECT_THAT(connectivity.GetConnectedLinkIndices(index),
                UnorderedElementsAreArray(all_link_indices));
  }
}

TEST(AssemblyConnectivityWithStringsTest,
     FindsConnectedLinksForOnlyFixedLinks) {
  CC_ASSERT_OK_AND_ASSIGN(
      const Assembly assembly,
      FromProto(ParseTextProtoOrDie<AssemblyProto>(kTestAssembly)));

  AssemblyConnectivityWithStrings connectivity(assembly,
                                               /*max_movable_joints=*/0);

  EXPECT_THAT(connectivity.GetConnectedLinkNames("no_such_link"), IsEmpty());
  EXPECT_THAT(connectivity.GetConnectedLinkNames("root"),
              UnorderedElementsAre("root", "A", "B", "C"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("A"),
              UnorderedElementsAre("A", "root", "C", "B"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("B"),
              UnorderedElementsAre("B", "root", "A", "C"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("C"),
              UnorderedElementsAre("C", "root", "A", "B"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("D"), ElementsAre("D"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("E"), ElementsAre("E"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("F"), ElementsAre("F"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("G"), ElementsAre("G"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("H"), ElementsAre("H"));
}

TEST(AssemblyConnectivityWithStringsTest,
     FindsConnectedLinksForOneMovableJoint) {
  CC_ASSERT_OK_AND_ASSIGN(
      const Assembly assembly,
      FromProto(ParseTextProtoOrDie<AssemblyProto>(kTestAssembly)));

  AssemblyConnectivityWithStrings connectivity(assembly,
                                               /*max_movable_joints=*/1);

  EXPECT_THAT(connectivity.GetConnectedLinkNames("no_such_link"), IsEmpty());
  EXPECT_THAT(connectivity.GetConnectedLinkNames("root"),
              UnorderedElementsAre("root", "A", "B", "C", "F", "E", "D"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("A"),
              UnorderedElementsAre("A", "root", "C", "B", "F", "E", "D"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("B"),
              UnorderedElementsAre("B", "root", "A", "C", "F", "E", "D"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("C"),
              UnorderedElementsAre("C", "root", "A", "B", "F", "E", "D"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("D"),
              UnorderedElementsAre("D", "C", "B", "A", "root"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("E"),
              UnorderedElementsAre("E", "B", "C", "A", "root"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("F"),
              UnorderedElementsAre("F", "G", "H", "root", "A", "B", "C"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("G"),
              UnorderedElementsAre("G", "F"));
  EXPECT_THAT(connectivity.GetConnectedLinkNames("H"),
              UnorderedElementsAre("H", "F"));
}

TEST(AssemblyConnectivityWithStringsTest,
     FindsAllIfMaxMovableJointsLargeEnough) {
  CC_ASSERT_OK_AND_ASSIGN(
      const Assembly assembly,
      FromProto(ParseTextProtoOrDie<AssemblyProto>(kTestAssembly)));

  std::vector<std::string> all_link_names;
  for (const auto& link : assembly.GetLinks()) {
    all_link_names.push_back(link.GetName());
  }
  AssemblyConnectivityWithStrings connectivity(
      assembly, /*max_movable_joints=*/all_link_names.size() - 1);

  for (const auto& name : all_link_names) {
    SCOPED_TRACE(absl::StrCat("name: ", name));
    EXPECT_THAT(connectivity.GetConnectedLinkNames(name),
                UnorderedElementsAreArray(all_link_names));
  }
}

}  // namespace
}  // namespace collision_checking
