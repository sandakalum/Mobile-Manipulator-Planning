#ifndef KNOWLEDGE_BASE_H
#define KNOWLEDGE_BASE_H

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace moveit_tmp
{

enum ObjectType
{
  MOVABLE,
  FIXED,
  SURFACE
};

struct Surface
{
  std::string frame_id;
  Eigen::Isometry3d p_min;
  Eigen::Isometry3d p_max;
};

struct Object
{
  std::string id;
  ObjectType type;
  bool has_surface;
  Surface surface;
};

using ObjectPtr = std::shared_ptr<Object>;

struct ObjectComparator
{
  bool operator()(const Object& lhs, const Object& rhs) const
  {
    return lhs.id < rhs.id;
  };
};

class KnowledgeBase
{
private:
  class On;
  class InHand;
  class Obstruct;
  class IsStackable;
  class LeaveClean;

public:
  KnowledgeBase();

  KnowledgeBase(const KnowledgeBase&);

  virtual ~KnowledgeBase() {}

  void addObject(const Object& object);

  void addObject(const std::string object_id, const ObjectType object_type);

  void removeObject(const std::string object_id);

  bool getObjectById(const std::string object_id, Object& object);

  std::set<Object>::iterator findObjectById(const std::string object_id);

  void reset();

  std::shared_ptr<On> getOn();
  std::shared_ptr<InHand> getInHand();
  std::shared_ptr<Obstruct> getObstruct();
  std::shared_ptr<IsStackable> getIsStackable();
  std::shared_ptr<LeaveClean> getLeaveClean();

  std::string getPDDL();

private:
  class Fact
  {
  public:
    Fact(KnowledgeBase* kb) : kb_(kb) {}
    virtual std::string getPDDL() = 0;
    virtual void removeObject(std::string object_id) = 0;
    virtual void reset() = 0;

  protected:
    KnowledgeBase* kb_;

  private:
    Fact();
  };

  class On : public Fact
  {
  public:
    On(KnowledgeBase* kb) : Fact(kb) {}

    void add(const std::string object_1, const std::string object_2);

    bool getUnderById(const std::string object_id, Object& on_object);

    std::string getPDDL() override;

    void removeObject(const std::string object_id) override;

    void reset() override;

  private:
    std::vector<std::pair<std::string, std::string>> on_;
  };

  class Obstruct : public Fact
  {
  public:
    Obstruct(KnowledgeBase* kb) : Fact(kb) {}

    void add(std::string obstruction, std::string obstructed);

    std::string getPDDL() override;

    void removeObject(std::string object_id) override;

    void reset() override;

  private:
    std::map<std::string, std::vector<std::string>> obstructions_;
  };

  class InHand : public Fact
  {
  public:
    InHand(KnowledgeBase* kb) : Fact(kb) {}

    void add(std::string object_id);

    void empty();

    void removeObject(std::string object_id) override;

    std::string getPDDL() override;

    void reset() override;

  private:
    std::string in_hand_;
  };

  class IsStackable : public Fact
  {
  public:
    IsStackable(KnowledgeBase* kb) : Fact(kb) {}

    void add(std::string object_id, bool stackable);

    std::string getPDDL() override;

    void removeObject(std::string object_id) override;

    void reset() override;

  private:
    std::vector<std::pair<std::string, bool>> stackables_;
  };

  class LeaveClean : public Fact
  {
  public:
    LeaveClean(KnowledgeBase* kb) : Fact(kb) {}

    void add(std::string object_1, std::string object_2);

    std::string getPDDL() override;

    void removeObject(std::string object_id) override;

    void reset() override;

  private:
    std::vector<std::pair<std::string, std::string>> leave_clean_;
  };

  std::set<Object, ObjectComparator> objects_;

  std::shared_ptr<On> on_;

  std::shared_ptr<InHand> in_hand_;

  std::shared_ptr<Obstruct> obstruct_;

  std::shared_ptr<IsStackable> is_stackable_;

  std::shared_ptr<LeaveClean> leave_clean_;
};

} // namespace moveit_tmp
#endif // KNOWLEDGE_BASE_H
